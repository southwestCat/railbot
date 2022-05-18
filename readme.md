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

5. 使用wpa_passphrase 生成连接wifi的配置文件，`wpa_passphrase [ssid] [password] >> file.config` 如wifi的ssid为tjrg3009，密码是 tjrg3009，保存配置文件到`~/wpa_supplicant.conf`
    ```bash
    wpa_passphrase tjrg3009 tjrg3009 >> /home/nao/wps_supplicant.conf
    ```
6. 使用`wpa_supplicant`连接`wifi`
   ```bash
   sudo wpa_supplicant -B -Dnl80211 -iwlan0 -c/home/nao/wpa_supplicant.conf
   ```
7. `ip addr` 查看无线网络是否连接成功，简单的测试一下网络连接
    ```bash
    sudo apt update
    ```
8. 这样nao机器人就有了和一般ubuntu系统几乎相同的体验，安装build-essential等都可以用apt安装。