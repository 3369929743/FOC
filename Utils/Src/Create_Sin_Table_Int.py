import math

table_size = 1024
two_pi = 2.0 * math.pi
pi_div_2 = math.pi / 2.0
sin_val_scale = 32767.0
angle_scale = 65536.0

# 生成 C 语言格式
with open("./Utils/Inc/sin_table.h", "w", encoding="utf-8") as f:
    f.write("#ifndef __SIN_TABLE_H__\n#define __SIN_TABLE_H__\n\n")
    f.write("#include <stdint.h>\n\n")  # 引入标准整型头文件

    f.write("// 自动生成的正弦表(Q15格式)，请勿手动修改\n")
    f.write(f"#define __2PI {two_pi:.7f}f\n")
    f.write(f"#define __PI_DIV_2 {pi_div_2:.7f}f\n")
    f.write(f"#define SIN_TABLE_SIZE {table_size}\n")
    f.write(f"#define SIN_VAL_SCALE {sin_val_scale}\n")
    f.write(f"#define ANGLE_SCALE ({angle_scale} / __2PI) \n")
    f.write(f"#define SIN_INT_SCALE (1 / SIN_VAL_SCALE) \n")
    f.write(f"#define INDEX_SCALE (SIN_TABLE_SIZE / __2PI)\n")
    f.write(f"#define TABLE_MASK (SIN_TABLE_SIZE - 1)\n\n")
    
    f.write(f"//正弦值的放大倍数为{sin_val_scale}\n\n")

    # 注意这里改成了 const int16_t
    f.write(f"const int16_t sin_table_int[SIN_TABLE_SIZE] = {{\n    ")
    
    for i in range(table_size):
        val = math.sin(i * two_pi / table_size)
        # 核心改动：乘以 32767 并强转为整数
        int_val = int(val * sin_val_scale) 
        f.write(f"{int_val}")
        
        # 逗号排版（整数不需要加 f 后缀）
        if i < table_size - 1:
            f.write(", ")
        if (i + 1) % 8 == 0:
            f.write("\n    ")
            
    f.write("\n};\n\n#endif\n")