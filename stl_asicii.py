from stl import mesh
from stl import Mode
import os

# 要转换的 STL 文件列表（原始路径：目标 ASCII 路径）
files = {
    'base_link': 'meshes/base_link.STL',
    'link1': 'meshes/link1.STL',
    'link2': 'meshes/link2.STL'
}

for name, path in files.items():
    ascii_path = f'meshes/{name}_ascii.STL'

    if not os.path.exists(path):
        print(f"[跳过] 找不到文件: {path}")
        continue

    try:
        print(f"[转换] {path} -> {ascii_path}")
        m = mesh.Mesh.from_file(path)
        m.save(ascii_path, mode=Mode.ASCII)

        # 再次检查文件开头是否含非法字节前缀（例如 b'xxxx'）
        with open(ascii_path, 'r') as f:
            content = f.read()

        if "b'" in content[:100]:  # 只检查前100个字符
            print(f"[清理] 检测到非法前缀，重新写入: {ascii_path}")
            content = content.split('\n', 1)[-1]  # 去除第一行
            with open(ascii_path, 'w') as f:
                f.write("solid " + name + "\n" + content)

    except Exception as e:
        print(f"[错误] 处理 {path} 失败: {e}")
