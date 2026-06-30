在.venv\Lib\site-packages\w̲e̲b̲o̲t̲s̲_controller.pth中写入

```
C:\Program Files\Webots\lib\controller\python3
```

在.venv\Lib\site-packages\sitecustomize.py中写入

``` python
import os
WEBOTS_HOME = os.path.expandvars(r"C:\Program Files\Webots")
```

注意其中提到的路径是你的webots的安装目录，这里是默认的