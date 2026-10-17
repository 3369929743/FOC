import os
import glob
import re

# 确保工作目录切换到工程根目录
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
os.chdir(project_root)

cmake_path = "CMakeLists.txt"
if not os.path.exists(cmake_path):
    print("【错误】未在当前目录下找到 CMakeLists.txt，请确保在 STM32 工程根目录下运行！")
    input("按回车键退出...")
    exit()

print("===== 开始扫描 STM32 自定义目录 =====")

# 1. 排除 STM32 官方默认生成的文件夹
exclude_dirs = {".git", ".vscode", "build", "Core", "Drivers", "cmake", ".settings", "Middlewares"}

user_sources_globs = []
user_include_paths = []

# 2. 扫描用户自定义的文件夹 (如 BSP, Utils, Device, APP 等)
for item in os.listdir("."):
    if os.path.isdir(item) and item not in exclude_dirs:
        # 扫描源文件目录 (如 BSP/Src)
        src_path = os.path.join(item, "Src")
        if os.path.exists(src_path):
            user_sources_globs.append(f'"{item}/Src/*.c"')
            print(f"  [+] 识别到源码目录: {item}/Src")
            
        # 扫描头文件目录 (如 BSP/Inc)
        inc_path = os.path.join(item, "Inc")
        if os.path.exists(inc_path):
            user_include_paths.append(f'"{item}/Inc"')
            print(f"  [+] 识别到头文件目录: {item}/Inc")

# 3. 读取并修改 CMakeLists.txt
with open(cmake_path, "r", encoding="utf-8") as f:
    content = f.read()

# 4. 自动确保开启 CMAKE_EXPORT_COMPILE_COMMANDS (IntelliSense 补全关键)
if "CMAKE_EXPORT_COMPILE_COMMANDS" not in content:
    # 插在 project(...) 命令后面
    match = re.search(r'project\([^)]+\)', content, re.IGNORECASE)
    if match:
        insert_pos = match.end()
        content = content[:insert_pos] + "\n\n# 自动开启编译数据库生成，用于 VS Code 完美代码补全\nset(CMAKE_EXPORT_COMPILE_COMMANDS ON)" + content[insert_pos:]
        print("  [✓] 已在 CMake 中开启 CMAKE_EXPORT_COMPILE_COMMANDS 补全支持")

# 5. 构造自动生成的 CMake 代码块
start_marker = "# [[[ STM32_AUTO_GENERATED_START ]]]"
end_marker = "# [[[ STM32_AUTO_GENERATED_END ]]]"

# 格式化输出到 CMake 的文本
sources_text = "\n    ".join(user_sources_globs)
includes_text = "\n    ".join(user_include_paths)

auto_block = f"""{start_marker}
# 此标记块由脚本自动维护，请勿手动修改

# 1. 递归查找自定义源文件
file(GLOB_RECURSE USER_SOURCES 
    {sources_text}
)

# 2. 将源文件添加到编译中
target_sources(${{CMAKE_PROJECT_NAME}} PRIVATE
    ${{USER_SOURCES}}
)

# 3. 添加自定义头文件包含路径
target_include_directories(${{CMAKE_PROJECT_NAME}} PRIVATE
    {includes_text}
)
{end_marker}"""

# 6. 替换或追加自动生成块
if start_marker in content and end_marker in content:
    # 如果已经存在，进行精准替换
    pattern = re.escape(start_marker) + r".*?" + re.escape(end_marker)
    content = re.sub(pattern, auto_block, content, flags=re.DOTALL)
    print("  [✓] 成功更新已存在的自定义构建块！")
else:
    # 如果不存在，追加到文件末尾
    content = content.rstrip() + "\n\n" + auto_block
    print("  [✓] 已在 CMakeLists.txt 末尾成功创建构建块！")

# 保存修改
with open(cmake_path, "w", encoding="utf-8") as f:
    f.write(content)

print("===== STM32 路径自动配置圆满完成！ =====")