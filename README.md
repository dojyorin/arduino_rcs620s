# RC-S620S FeliCa Reader/Writer for Arduino

**WIP!! 作業中のため動作未確認となります**

ArduinoからRC-S620/Sモジュールを操作するためのライブラリです。

# 仕様
## クラス
### `RCS620S`

```arduino
RCS620S nfc;
```

## プロパティ
### `uint32_t timeout`
RC-S620/S との通信のタイムアウトをミリ秒単位で指定します。

- デフォルト値: `1000`

### `uint8_t idm[8]`
捕捉したカードのIDmです。

### `uint8_t pmm[8]`
捕捉したカードのPMmです。

## メソッド
### `bool begin(HardwareSerial* uart)`
RC-S620/Sの初期化を行います。

任意の`HardwareSerial`を初期化し、オブジェクトのアドレスを代入してください。

```arduino
RCS620S nfc;
Serial1.begin(115200);
nfc.begin(&Serial1);
```

### `bool polling(uint16_t code)`
指定されたシステムコードを持つ FeliCa カードの捕捉を試みます。
捕捉に成功した場合は、プロパティ idm と pmm に捕捉したカードの IDm および PMm がそれぞれ設定されます。

この関数を呼び出すと RC-S620/S から搬送波の出力が開始されます。
搬送波の出力を止めたいときは、rfOff() を呼び出してください。

### `bool cardCommand(const uint8_t* cmd, uint8_t cmdLen, uint8_t res[CardResponseMax], uint8_t* resLen)`
polling() で捕捉したカードにカードコマンドを送信し、レスポンスを受信します。

送信するコマンドを`cmd`に、その長さを`cmdLen`に指定してください。

カードレスポンスの受信に成功すると、受信したレスポンスが`res`に、その長さが`resLen`に格納されます。

送受信できるコマンドおよびレスポンスの最大長は`254`バイトです。

この関数は`polling()`が成功したあとに呼び出してください。

### `bool push(const uint8_t* data, uint8_t dataLen)`
三者間通信コマンド(PushおよびActivate2)を使用して、モバイルFeliCa ICチップ搭載機器などに指定されたデータを送信します。
送信するデータを`data`に、その長さを`dataLen`に指定してください。

この関数は`polling()`が成功したあとに呼び出してください。

### `bool close()`
搬送波の出力を停止します。