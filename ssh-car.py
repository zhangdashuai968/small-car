"""SSH 小车工具 - 执行远程命令或进入交互式 shell"""
import paramiko
import sys
import os

HOST = "192.168.36.46"
USER = "abot"
PASSWORD = "123456"


def ssh_command(cmd: str) -> str:
    """在车上执行命令并返回输出"""
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(HOST, username=USER, password=PASSWORD, timeout=10)
    stdin, stdout, stderr = ssh.exec_command(cmd)
    out = stdout.read().decode()
    err = stderr.read().decode()
    ssh.close()
    if err:
        return f"[stderr]\n{err}\n[stdout]\n{out}"
    return out


def ssh_interactive():
    """交互式 shell（Windows 终端）"""
    import subprocess
    print(f"正在连接 {USER}@{HOST} ...")
    # 使用 Windows 原生 ssh，需要先安装 sshpass 或手动输密码
    # 这里用 paramiko 模拟一个简单的交互循环
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(HOST, username=USER, password=PASSWORD, timeout=10)
    print(f"已连接到 {HOST}，输入命令执行，输入 exit 退出。")
    while True:
        try:
            cmd = input(f"\n{USER}@{HOST}:~$ ")
        except (EOFError, KeyboardInterrupt):
            print("\n断开连接。")
            break
        if cmd.strip() == "exit":
            print("断开连接。")
            break
        if not cmd.strip():
            continue
        stdin, stdout, stderr = ssh.exec_command(cmd)
        out = stdout.read().decode()
        err = stderr.read().decode()
        if err:
            print(err, end="")
        if out:
            print(out, end="")
    ssh.close()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        # 执行单条命令
        cmd = " ".join(sys.argv[1:])
        print(ssh_command(cmd))
    else:
        ssh_interactive()
