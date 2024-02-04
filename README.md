# 情報メディア実験B ライントレーサ製作

### これは何?
情報メディア創成学類3年次の情報メディア実験Bの題目の1つである、ライントレーサ製作に使用したソースコードです。

### 構成
2つのファイルに分かれています。

- #### main.ino
メインのソースコードです。PID制御などはすべてこちらに書かれています。

- #### sensor.ino
センサの値を統合してメインに渡すためのソースコードです。
センサの処理とメインの処理でマイコンを分けた関係でソースコードも分かれています。