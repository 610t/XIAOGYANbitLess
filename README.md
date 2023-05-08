# XIAOGYANbitLess
![XIAOGYANbitLess](https://i.gyazo.com/3ea76b74d558eccc9f6099eac66ffeb1.jpg)

XIAOGYANbitLessは、[XIAOGYAN8周年記念ボード](https://github.com/algyan/XIAOGYAN)をScratchで使うためのプログラムです。

Scratchには、Microbit More拡張機能が利用可能な、([Stretch3](https://stretch3.github.io/))などで利用が可能です。

## 対応拡張ボード
- [XIAOGYAN8周年記念ボード](https://github.com/algyan/XIAOGYAN)
- [Seeed Studio XIAO 拡張ボード](https://wiki.seeedstudio.com/Seeeduino-XIAO-Expansion-Board)

### 現在利用できるデバイス
現在、以下のようなデバイスに対応しています。

#### XIAOGYAN8周年記念ボード
- 5x5 LEDとしてのパターン表示
- 8x8 LEDへの文字列表示
- I2C加速度計 (ADXL345 or MPU6886)
- ボタンスイッチ
- スピーカーからのトーン音出力
- P0としてのアナログ入力(XIAOGYANのA3ポート)

#### Seeed Studio XIAO 拡張ボード
- 5x5 LEDとしての表示
- I2C加速度計 (MPU6886 or ADXL345)
- 文字列の表示

## 使用方法
XIAOGYANボード側とScratch側で以下のように準備を行います。

### XIAOGYANボード側
XIAOGYANbitLessをコンパイルして、ボードに書き込みます。
ArduinoIDEでも、PlatformIOでもコンパイルできます。

### Scratch (Stretch3側)
Scratchは、Microbit More拡張機能が利用可能な[Stretch3](https://stretch3.github.io/)を使ってください。
Stretch3で、以下のように操作をします。

![拡張機能](https://i.gyazo.com/208ad9cd788d453555267d8901b4050b.png)
一番左下のボタンから拡張機能の追加を行います。

![Microbit Moreの選択](https://i.gyazo.com/4780d7b0da3a260f7e709db4b16334c3.png)

Microbit Moreの拡張機能を追加します。

![デバイスの選択](https://i.gyazo.com/be6c3374e86301eb7874fa0d1ba9575d.png)

Microbit More拡張機能ブロックカテゴリの右上黄色の(!)を押して、接続するデバイスを選択します。

あとは、通常のMicrobit Moreのように利用することができますが、まだ実装していない機能があるので注意してください。
