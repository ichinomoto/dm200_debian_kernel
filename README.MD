# dm200_debian_kernel
提供されたソースは、このリポジトリの初期コミット。  
defconfigは `onyx2_warp_defconfig`

## 主な変更点
* FrameBufferの初期化処理変更
* キーリピート有効化
* suspend時に各種デバイスの電源が切れるように処理を強制的に変更
* USB-HOST時にVBUS出力しないように変更
* キーボードのキー定義変更
* Framebufferの色深度を32bitで初期化
* カーネルコンフィグの変更多数
* /dev/uinputを有効化

## コンパイル
Ubuntu14.04環境で (16.04ではaptで入るgccが新しすぎるため、14.04を使うほうが楽)
```
sudo apt install gcc-arm-linux-gnueabihf
sudo apt install bc lzop
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabihf-
make defconfig dm200_defconfig
make dep
make zImage
./mkkrnlimg arch/arm/zImage kernel.img
```
