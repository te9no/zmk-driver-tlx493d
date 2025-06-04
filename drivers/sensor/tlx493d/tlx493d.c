#define DT_DRV_COMPAT infineon_tlx493d // デバイスツリーの互換性文字列

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

// ログモジュールを登録。第二引数でデフォルトのログレベルを指定できます。
// KconfigでCONFIG_SENSOR_LOG_LEVEL_DBG=yなどを設定すると、LOG_DBGが有効になります。
LOG_MODULE_REGISTER(tlx493d, CONFIG_SENSOR_LOG_LEVEL);

// センサー固有のレジスタアドレスなど (データシートで確認してください)
#define TLX493D_REG_READ_FRAME_0  0x00 // 例: データフレームの開始レジスタ
// ... その他必要なレジスタ定義 (例: 設定レジスタなど)
#define TLX493D_EXPECTED_BYTES    7    // 例: 読み出すバイト数 (データシートで確認)

struct tlx493d_data {
    const struct device *dev;
    struct k_work_delayable work;
    struct k_timer timer;

    const struct i2c_dt_spec i2c;

    int16_t prev_x;
    int16_t prev_y;
    int16_t prev_z; // Z軸も差分計算する場合
};

struct tlx493d_config {
    uint32_t polling_interval_ms;
};

// TLX493Dからセンサーデータを読み出す関数
static int tlx493d_read_sensor_data(const struct device *dev, int16_t *x, int16_t *y, int16_t *z) {
    LOG_DBG("Attempting to read sensor data...");
    struct tlx493d_data *data = dev->data;
    uint8_t buf[TLX493D_EXPECTED_BYTES]; // データシートに基づいてサイズを決定

    // TLX493Dのデータ読み出しシーケンス (データシート参照)
    if (i2c_read_dt(&data->i2c, buf, sizeof(buf)) < 0) { // アドレス指定なしで連続読み出しする場合など
    // if (i2c_burst_read_dt(&data->i2c, TLX493D_REG_READ_FRAME_0, buf, sizeof(buf)) < 0) { // 開始レジスタを指定する場合
        LOG_ERR("Failed to read sensor data via I2C");
        return -EIO;
    }

    // ログに生のバイト列を出力 (デバッグに役立つ場合)
    // LOG_HEXDUMP_DBG(buf, sizeof(buf), "Raw I2C data:");

    // データシートに基づいてバイト列を16ビット整数に変換
    // これはTLX493D-W2B6のデータフォーマットの一般的な解釈例です。
    // 必ずお使いのセンサーのデータシートで正確なフォーマットを確認してください。
    // 一般的に、最初の3ビットがX、次の3ビットがY、というわけではなく、
    // 各軸のデータが特定のバイトに格納され、ビットシフトやマスキングが必要です。
    // 以下は非常に単純化された仮の例です。実際のセンサーに合わせてください。

    // 例: (TLX493D-A0/A1/W2B6などはもう少し複雑なビットフィールドを持っています)
    // バイト0-1: X軸データ, バイト2-3: Y軸データ, バイト4-5: Z軸データ (と仮定)
    // buf[0]の上位NビットがX、buf[0]の下位Mビットとbuf[1]がX、など
    // 正確な変換ロジックはデータシートの「Register Description」や「Measurement results」セクションを参照

    // 仮の変換 (実際にはデータシートに従ってください！)
    *x = (int16_t)((buf[0] << 8) | buf[1]); // X軸
    *y = (int16_t)((buf[2] << 8) | buf[3]); // Y軸
    *z = (int16_t)((buf[4] << 8) | buf[5]); // Z軸

    LOG_DBG("Parsed sensor values - X: %d, Y: %d, Z: %d", *x, *y, *z);
    return 0;
}

// ワークキューコールバック関数
static void tlx493d_work_cb(struct k_work *work) {
    LOG_DBG("Work callback started");
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tlx493d_data *data = CONTAINER_OF(dwork, struct tlx493d_data, work);
    const struct device *dev = data->dev;
    const struct tlx493d_config *cfg = dev->config;

    int16_t current_x, current_y, current_z;
    int ret;

    ret = tlx493d_read_sensor_data(dev, &current_x, &current_y, &current_z);
    if (ret < 0) {
        LOG_ERR("Failed to read sensor data in work_cb, rescheduling.");
        goto reschedule;
    }

    // 初回実行時は差分計算をスキップ (または初期値を0とする)
    // INT16_MIN はあくまで仮の未初期化値。より確実な初回フラグを推奨。
    if (data->prev_x == INT16_MIN && data->prev_y == INT16_MIN) {
        LOG_DBG("First run, initializing previous values.");
        data->prev_x = current_x;
        data->prev_y = current_y;
        data->prev_z = current_z; // Z軸も使用する場合
        goto reschedule;
    }

    int16_t dx = current_x - data->prev_x;
    int16_t dy = current_y - data->prev_y;
    int16_t dz = current_z - data->prev_z;

    LOG_DBG("Calculated differences - dX: %d, dY: %d, dZ: %d", dx, dy, dz);

    // TODO: デッドゾーン処理や感度調整
    // 例:
    // #define DEAD_ZONE 5
    // if (abs(dx) < DEAD_ZONE) dx = 0;
    // if (abs(dy) < DEAD_ZONE) dy = 0;
    //
    // #define SENSITIVITY_DIVIDER 2
    // dx /= SENSITIVITY_DIVIDER;
    // dy /= SENSITIVITY_DIVIDER;

    bool reported = false;
    if (dx != 0) {
        input_report_rel(dev, INPUT_REL_X, dx, K_NO_WAIT);
        LOG_DBG("Reported INPUT_REL_X: %d", dx);
        reported = true;
    }
    if (dy != 0) {
        input_report_rel(dev, INPUT_REL_Y, dy, K_NO_WAIT);
        LOG_DBG("Reported INPUT_REL_Y: %d", dy);
        reported = true;
    }

    if (dz != 0) {
       input_report_rel(dev, INPUT_REL_WHEEL, dz, K_NO_WAIT);
       LOG_DBG("Reported INPUT_REL_WHEEL: %d", dz);
       reported = true;
    }

    if (reported) {
        LOG_DBG("Calling input_sync()");
        input_sync(dev);
    }

    data->prev_x = current_x;
    data->prev_y = current_y;
    data->prev_z = current_z;

reschedule:
    LOG_DBG("Rescheduling work in %d ms", cfg->polling_interval_ms);
    k_work_reschedule(&data->work, K_MSEC(cfg->polling_interval_ms));
}

// タイマーハンドラ
static void tlx493d_timer_handler(struct k_timer *timer) {
    struct tlx493d_data *data = CONTAINER_OF(timer, struct tlx493d_data, timer);
    LOG_DBG("Timer expired, scheduling work callback.");
    // ワークキューをスケジュール (システムワークキューで即時実行を試みる)
    k_work_reschedule(&data->work, K_NO_WAIT);
}

// 初期化関数
static int tlx493d_init(const struct device *dev) {
    LOG_DBG("Initializing TLX493D driver for device: %s", dev->name);
    struct tlx493d_data *data = dev->data;
    const struct tlx493d_config *cfg = dev->config;

    data->dev = dev;
    // 初回判定用の初期値を設定 (より堅牢なフラグ管理も検討)
    data->prev_x = INT16_MIN;
    data->prev_y = INT16_MIN;
    data->prev_z = INT16_MIN;

    if (!device_is_ready(data->i2c.bus)) {
        LOG_ERR("I2C bus %s is not ready.", data->i2c.bus->name);
        return -ENODEV;
    }
    LOG_DBG("I2C bus %s is ready.", data->i2c.bus->name);

    // TODO: TLX493Dセンサーの初期化シーケンス (データシートに基づいたレジスタ設定)
    // 例: センサーをアクティブモードにする、測定範囲を設定する、フィルターを設定するなど
    // uint8_t init_payload[] = {REGISTER_ADDR, VALUE_TO_WRITE};
    // if (i2c_write_dt(&data->i2c, init_payload, sizeof(init_payload)) < 0) {
    //     LOG_ERR("Failed to write initial configuration to sensor.");
    //     return -EIO;
    // }
    // LOG_DBG("Sensor initial configuration written.");

    k_work_init_delayable(&data->work, tlx493d_work_cb);
    k_timer_init(&data->timer, tlx493d_timer_handler, NULL);

    // タイマーを開始して、指定されたポーリング間隔で定期的に起動
    // 最初の呼び出しもポーリング間隔後になる
    k_timer_start(&data->timer, K_MSEC(cfg->polling_interval_ms), K_MSEC(cfg->polling_interval_ms));
    LOG_DBG("Timer started with interval: %d ms", cfg->polling_interval_ms);

    // 起動直後に一度データを読み込みたい場合は、ここで直接ワークキューをスケジュール
    // k_work_reschedule(&data->work, K_MSEC(50)); // 50ms後に最初の読み込みなど

    LOG_INF("TLX493D driver initialized successfully for %s", dev->name);
    return 0;
}

static const struct input_device_api tlx493d_input_api = {};

#define TLX493D_INIT(inst)                                               \
    static struct tlx493d_data tlx493d_data_##inst = {                  \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                             \
    };                                                                   \
                                                                        \
    static const struct tlx493d_config tlx493d_config_##inst = {        \
        .polling_interval_ms = DT_INST_PROP(inst, polling_interval),    \
    };                                                                   \
                                                                        \
    DEVICE_DT_INST_DEFINE(inst,                                         \
                         tlx493d_init,                                   \
                         NULL,                                           \
                         &tlx493d_data_##inst,                          \
                         &tlx493d_config_##inst,                         \
                         POST_KERNEL,                                    \
                         CONFIG_INPUT_INIT_PRIORITY,                     \
                         &tlx493d_input_api);

DT_INST_FOREACH_STATUS_OKAY(TLX493D_INIT)