import math

table_size = 1024
two_pi = 2.0 * math.pi
pi_div_2 = math.pi / 2.0

# 生成 C 语言格式
with open("./Utils/Inc/sin_table.h", "w", encoding="utf-8") as f:
    f.write("#ifndef __SIN_TABLE_H__\n#define __SIN_TABLE_H__\n\n")
    f.write("// 自动生成的正弦表，请勿手动修改\n")
    f.write(f"#define __2PI {two_pi:.7f}f\n")
    f.write(f"#define __PI_DIV_2 {pi_div_2:.7f}f\n")
    f.write(f"#define SIN_TABLE_SIZE {table_size}\n")
    f.write(f"#define INDEX_SCALE (SIN_TABLE_SIZE / __2PI)\n")
    f.write(f"#define TABLE_MASK (SIN_TABLE_SIZE - 1)\n")
    f.write(f"const float sin_table[SIN_TABLE_SIZE] = {{\n    ")
    
    for i in range(table_size):
        val = math.sin(i * two_pi / table_size)
        # 保留 8 位小数，足够精确
        f.write(f"{val:.8f}f")
        if i < table_size - 1:
            f.write(", ")
        if (i + 1) % 8 == 0:
            f.write("\n    ")
            
    f.write("\n};\n\n#endif\n")