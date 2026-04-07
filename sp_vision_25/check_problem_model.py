import openvino.runtime as ov
import sys

def check_specific_model(model_path):
    print(f"\n详细检查模型: {model_path}")
    try:
        core = ov.Core()
        print("1. 创建 Core 对象成功")
        
        # 尝试加载模型
        print(f"2. 尝试读取模型...")
        model = core.read_model(model_path)
        print("3. 读取模型成功")
        
        print(f"4. 模型输入: {[inp.get_any_name() for inp in model.inputs]}")
        print(f"5. 模型输出: {[out.get_any_name() for out in model.outputs]}")
        
        # 尝试编译模型
        print("6. 尝试编译模型...")
        compiled_model = core.compile_model(model, "CPU")
        print("7. 模型编译成功！")
        
        return True
    except Exception as e:
        print(f"✗ 错误发生在: {e}")
        import traceback
        traceback.print_exc()
        return False

# 检查出问题的模型
models = ["assets/yolo11_buff_int8.xml"]

for model in models:
    if check_specific_model(model):
        print(f"\n✓ {model} 检查通过")
    else:
        print(f"\n✗ {model} 检查失败")
    print("=" * 60)
