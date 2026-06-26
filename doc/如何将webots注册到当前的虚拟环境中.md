在 Windows 上，Webots 的 Python API（controller、vehicle 等）并非通过 PyPI 分发，而是随软件安装在本地目录中。要将其"注册"到 uv 管理的虚拟环境，最可靠的方式是借助 .pth 文件把 Webots 的 Python 库路径注入虚拟环境的 sys.path，同时解决 Windows 下 DLL 加载的依赖问题。
1. 确认 Webots Python 库路径
Webots 的 Python 控制器库通常位于安装目录下的版本相关子目录中，例如：
Text

C:\Users\<用户名>\AppData\Local\Programs\Webots\lib\controller\python311

或（若安装在系统目录）：
Text

C:\Program Files\Webots\lib\controller\python311

注意：路径中的 python311 需与你 uv 虚拟环境的 Python 版本严格匹配（如 3.11 对应 python311，3.10 对应 python310）。
2. 创建 .pth 文件注入路径
uv 的虚拟环境 site-packages 目录通常位于项目根目录的 .venv\Lib\site-packages 下。你可以直接创建一个 .pth 文件指向 Webots 库：
powershell

# PowerShell 示例，假设 Webots 安装在用户目录，且 uv 虚拟环境已创建
$WEBOTS_PYTHON = "$env:LOCALAPPDATA\Programs\Webots\lib\controller\python311"
$SITE_PACKAGES = uv run python -c "import site; print(site.getsitepackages()[0])"

# 创建 .pth 文件
Set-Content -Path "$SITE_PACKAGES\webots_controller.pth" -Value $WEBOTS_PYTHON

这等价于 uv 在处理 uv add --editable 时自动生成的 _<package>.pth 机制，只是这里我们手动为外部非标准包创建。
3. 解决 Windows DLL 加载问题（关键）
仅添加 .pth 文件在 Windows 上通常不够。Webots 的 _controller.pyd 等扩展模块依赖 Controller.dll 以及 MinGW 运行时 DLL，而 Windows 不会从 .pth 路径自动解析这些 DLL，直接 import controller 会报 ImportError: DLL load failed。
推荐在虚拟环境的 site-packages 下创建一个 sitecustomize.py，在 Python 启动时自动注册 DLL 搜索路径：
Python

# 保存到 .venv\Lib\site-packages\sitecustomize.py
import os
import sys

WEBOTS_HOME = os.path.expandvars(r"$LOCALAPPDATA\Programs\Webots")  # 按实际安装路径修改

# Python 3.8+ 的 Windows DLL 目录机制
if hasattr(os, 'add_dll_directory') and os.name == 'nt':
    controller_dll = os.path.join(WEBOTS_HOME, "lib", "controller")
    mingw_dll = os.path.join(WEBOTS_HOME, "msys64", "mingw64", "bin")
    
    for p in [controller_dll, mingw_dll]:
        if os.path.exists(p):
            os.add_dll_directory(p)

# 同时确保环境变量中 WEBOTS_HOME 存在，供 Webots API 内部使用
os.environ.setdefault("WEBOTS_HOME", WEBOTS_HOME)

    替代方案：如果你不想动虚拟环境内部文件，也可以在每次运行前通过 PowerShell 设置 Path 环境变量：
    powershell

    $env:WEBOTS_HOME = "$env:LOCALAPPDATA\Programs\Webots"
    $env:Path = "$env:WEBOTS_HOME\lib\controller;$env:WEBOTS_HOME\msys64\mingw64\bin;" + $env:Path
    uv run python your_script.py

4. 验证是否成功
powershell

uv run python -c "from controller import Robot; print('Webots controller imported successfully')"

如果无报错，说明包已正确注册且 DLL 依赖已解决。