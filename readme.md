# Railbot

## 准备工作
1. 编译[BHumanCodeRelease2021](https://github.com/bhuman/BHumanCodeRelease)代码，下载的时候需要使用`--recursive`，编译方法见[public wiki](https://wiki.b-human.de/coderelease2021/)
    ```bash
    git clone --recursive https://github.com/bhuman/BHumanCodeRelease.git
    ```


2. 使用flasher将bhuman.opn刷机到Nao机器人上，默认的IP是`10.1.89.40`

3. 刷机后，需要手动开机，电脑设置静态IP为`10.1.100.10`后两位可以随便设置，ssh连接机器人，停用bhuman程序，禁止开机自启动
    ```bash
    systemctl --user stop bhuman.service
    systemctl --user disable bhuman.service
    ```
    一些常用的`systemctl`指令，默认 sudo 密码为 nao

    关机
    ```bash
    sudo systemctl poweroff
    ```
    
    重启
    ```bash
    sudo systemctl reboot
    ```

4. 修改`/etc/netpln/default.yaml`，修改有线连接固定ip，添加wlan0为dhcp.
    ```bash
    nao@Default:~$ cat /etc/netplan/default.yaml
    network:
    version: 2
    renderer: networkd
    ethernets:
        eth0:
        addresses: [192.168.20.44/16]

        wlan0:
        addresses: []
        dhcp4: true
        optional: true
    nao@Default:~$ sudo netplan try
    nao@Default:~$ sudo netplan apply
    ```

5. 使用wpa_passphrase 生成连接wifi的配置文件，`wpa_passphrase [ssid] [password] >> file.config` 如wifi的ssid为 wifi-test，密码是 12345678，保存配置文件到`~/wpa_supplicant.conf`
    ```bash
    wpa_passphrase wifi-test 12345678 >> /home/nao/wps_supplicant.conf
    ```
6. 使用`wpa_supplicant`连接`wifi`
   ```bash
   sudo wpa_supplicant -B -Dnl80211 -iwlan0 -c/home/nao/wpa_supplicant.conf
   ```
7. 稍等一段时间后，使用`ip addr` 查看无线网络是否连接成功，简单的测试一下网络连接
    ```bash
    sudo apt update
    ```
8. 这样nao机器人就有了和一般ubuntu系统几乎相同的体验，安装build-essential等都可以用apt安装。

## 运行程序

1. [eigen-quadprog](https://github.com/jrl-umi3218/eigen-quadprog) 复制到机器人 /usr/local/lib/ 目录下。该库文件可以先在PC上编译好， 在机器上运行可执行文件提醒缺少库文件，ldd 查看链接库，再查看机器人缺少哪些库文件，然后复制到对应路径。Ubuntu20.04编译程序可以在nao-bhkernel上运行，其他版本的操作系统没有测试。
2. 在 `/home/nao/.bashrc` 末尾添加 
    ```bash
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
    ```
3. 在/home/nao目录，新建文件夹railbot，将Config文件夹，Bin/controller和Bin/lola_conn复制到/home/nao/railbot文件夹中，在终端中运行 lola_conn，新建终端运行 controller.