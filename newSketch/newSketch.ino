#include <M5StickC.h>

#include "BluetoothSerial.h" //Bluetooth用
BluetoothSerial SerialBT;    // Bluetooth用

hw_timer_t *timer;
QueueHandle_t xQueue;
TaskHandle_t taskHandle;

//ライトの取得
int lightSensor(int analogRead_value)
{
    // analogRead_valueの数値が2000以下 -> 明るい
    if (analogRead_value <= 2000)
    {
        M5.Lcd.setCursor(10, 10);
        M5.Lcd.printf("%d\n", analogRead_value);
        return 0;
    }
    // analogRead_valueの数値が2000以上 -> 暗い
    else
    {
        M5.Lcd.setCursor(10, 10);
        M5.Lcd.printf("%d\n", analogRead_value);
        return 1;
    }
}

// タイマー割り込み
void IRAM_ATTR onTimer()
{
    int8_t data;
    // キューを送信
    xQueueSendFromISR(xQueue, &data, 0);
}
// 実際のタイマー処理用タスク
void task(void *pvParameters)
{
    uint8_t calibration = 0;       // キャリブレーションの状態(0:初期化直後, 1:データ取得中, 2:完了)
    uint16_t calibrationCount = 0; // データ取得数
    float pitchSum = 0;            // ピッチの累計数
    float rollSum = 0;             // ロールの累計数
    float yawSum = 0;              // ヨーの累計数
    float pitchOffset = 0;         // ピッチのオフセット
    float rollOffset = 0;          // ロールのオフセット
    float yawOffset = 0;           // ヨーのオフセット
    float yawOld = 0;              // 前回のヨー
    float yaw2 = 0;                // 補正後のヨー

    float before_pitch = 0.0f;
    float before_roll = 0.0f;
    float before_yaw = 0.0f;
    
    while (1)
    {
        int8_t data;
        float pitch;
        float roll;
        float yaw;
        float accX;
        float accY;
        float accZ;
        // タイマー割り込みがあるまで待機する
        xQueueReceive(xQueue, &data, portMAX_DELAY);
        M5.IMU.getAhrsData(&pitch, &roll, &yaw);

        //================================================================================================
        // 最初の200個は読み捨てる #1000
        //================================================================================================
        
        if (calibration == 0)
        {
            calibrationCount++;
            if (1000 <= calibrationCount)
            {
                calibration = 1;
                calibrationCount = 0;
                yawOld = yaw;
            }
        }

        else if (calibration == 1)
        {
            //================================================================================================
            //　 一定時間データを取得してオフセットを計算する
            //================================================================================================
            float gyroX = 0.0F;
            float gyroY = 0.0F;
            float gyroZ = 0.0F;
            M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
            M5.MPU6886.getAccelData(&accX, &accY, &accZ);  //加速度取得
            float gyro = abs(gyroX) + abs(gyroY) + abs(gyroZ);
            if (25 < gyro)
            {
                // 振動があった場合には再度キャリブレーション
                calibrationCount = 0;
                pitchSum = 0;
                rollSum = 0;
                yawSum = 0;
//                Serial.printf("Calibration Init!!!!! %f\n", gyro);
            }
            else
            {
                // 累計を保存
                pitchSum += pitch;
                rollSum += roll;
                yawSum += (yawOld - yaw); // ヨーは差分を累計する
                yawOld = yaw;
                calibrationCount++;
                if (500 <= calibrationCount)
                {
                    // 一定数溜まったらオフセット計算
                    calibration = 2;
                    pitchOffset = pitchSum / calibrationCount;
                    rollOffset = rollSum / calibrationCount;
                    yawOffset = yawSum / calibrationCount;
//                    Serial.printf(" pitchSum = %f\n", pitchSum);
//                    Serial.printf(" rollSum = %f\n", rollSum);
//                    Serial.printf(" yawSum = %f\n", yawSum);
//                    Serial.printf(" pitchOffset = %f\n", pitchOffset);
//                    Serial.printf(" rollOffset = %f\n", rollOffset);
//                    Serial.printf(" yawOffset = %f\n", yawOffset);
                }
            }
        }
        else
        {
            //================================================================================================
            //　補正部分 #0.9/0.999999
            //================================================================================================

            pitch -= pitchOffset;
            roll -= rollOffset;
            float yawDelta = yawOld - yaw;
            yawOld = yaw;
            if (-200 < yawDelta && yawDelta < 200)
            {
                // 急激な変化は補正しない
                yaw2 -= yawDelta - yawOffset;
                // 10%減少させて原点に戻す
                yaw2 = yaw2 * 0.999999;
            }


            //================================================================================================
            //　出力部分
            //================================================================================================


//            Serial.printf(" %f,%f,%f\n", pitch, roll, yaw2);
            M5.Lcd.printf("%.2f  %.2f  %.2f\n", pitch, roll, yaw2);
            //シリアル通信
//            SerialBT.printf("x%.2fy%.2fz%.2f", pitch, roll, yaw2);
//            Serial.printf("x%.2fy%.2fz%.2f", pitch, roll, yaw2);
//            SerialBT.printf("a%.2fb%.2fc%.2f", accX, accY, accZ);
//            Serial.printf("a%.2fb%.2fc%.2f", accX, accY, accZ);

            //光センサー(数字が小さいと明るい, 数字が大きいと暗い)
            uint16_t analogRead_value = analogRead(33);
            int isLightCheck = lightSensor(analogRead_value);
            // シリアルモニタに”1"と文字列を送信
//            SerialBT.printf("l%d\n", isLightCheck);
//            Serial.printf("l%d\n", isLightCheck);

            float difference_pitch = pitch - before_pitch;
            float difference_roll = roll - before_roll ;
            float difference_yaw = yaw2 - before_yaw ;

//            SerialBT.printf("x%.2f,y%.2f,z%.2f", difference_pitch, difference_roll, difference_yaw);

            Serial.printf("x%.2f,y%.2f,z%.2f,a%.2f,b%.2f,c%.2f,l%d\r\n", difference_pitch, difference_roll, difference_yaw, accX, accY, accZ, isLightCheck);
            SerialBT.printf("x%.2f,y%.2f,z%.2f,a%.2f,b%.2f,c%.2f,l%d\r\n", difference_pitch, difference_roll, difference_yaw, accX, accY, accZ, isLightCheck);
            //Serial.printf("x%.2f,y%.2f,z%.2f,a%.2f,b%.2f,c%.2f,l%d\n", pitch, roll, yaw2, accX, accY, accZ, isLightCheck);
            //SerialBT.printf("x%.2f,y%.2f,z%.2f,a%.2f,b%.2f,c%.2f,l%d\n", pitch, roll, yaw2, accX, accY, accZ, isLightCheck);
            digitalWrite(10, isLightCheck); //LED をオンオフさせる。


            before_pitch = pitch;
            before_roll = roll;
            before_yaw = yaw2;
            

            //グラフ化
            String str = "pitch:"+String(pitch) + "," +"roll:" + String(roll) + "," + "yaw:" + String(yaw2);
//            Serial.println(str);
            //SerialBT.println(str);
            


        }
    }
}


void setup()
{
    M5.begin();
    M5.IMU.Init();
    M5.Lcd.setRotation(3); //モニタ画面方向設定
    M5.MPU6886.Init();     //センサ初期化
    Serial.begin(9600);  //シリアル通信初期化
    //Serial.begin(115200); //シリアル通信初期化
    //Serial.begin(4800);
    pinMode(32, INPUT);   //光センサーの読み取り
    pinMode(10, OUTPUT);  // LED

    // Bluetooth初期化（デバイス名）
    SerialBT.begin("M5STICK-C-TGSPET1");

    // キュー作成
    xQueue = xQueueCreate(1, sizeof(int8_t));
    // Core1の優先度5でタスク起動
    xTaskCreateUniversal(
        task,        // タスク関数
        "task",      // タスク名(あまり意味はない)
        8192,        // スタックサイズ
        NULL,        // 引数
        5,           // 優先度(大きい方が高い)
        &taskHandle, // タスクハンドル
        APP_CPU_NUM  // 実行するCPU(PRO_CPU_NUM or APP_CPU_NUM)
    );
    // 4つあるタイマーの1つめを利用
    // 1マイクロ秒ごとにカウント(どの周波数でも)
    // true:カウントアップ
    timer = timerBegin(0, getApbFrequency() / 1000000, true);
    // タイマー割り込み設定
    timerAttachInterrupt(timer, &onTimer, true);
    // マイクロ秒単位でタイマーセット
    timerAlarmWrite(timer, 10 * 1000, true);
    // タイマー開始
    timerAlarmEnable(timer);
}

void loop()
{
    // メイン処理は無し
    delay(1);

    M5.Lcd.fillScreen(BLACK);

    if(M5.BtnB.wasPressed()){
      esp_restart();
    }
    M5.update();
}
