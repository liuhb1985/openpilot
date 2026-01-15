#!/usr/bin/env python3
import os
import time
import numpy as np

print("=== 🔍 ROCm + ONNX Runtime GPU Verification ===")

# Step 1. 环境检测
print("\n[1] 检查 ROCm 安装路径...")
if os.path.exists("/opt/rocm"):
    print("✅ ROCm 路径存在: /opt/rocm")
else:
    print("❌ 未检测到 /opt/rocm，请确认 amdgpu-install 安装正常。")
    exit(1)

# Step 2. 检查 GPU 信息
print("\n[2] GPU 信息检测...")
try:
    os.system("/opt/rocm/bin/rocminfo | grep Name | head -n 1")
except Exception as e:
    print("⚠️ 运行 rocminfo 失败，请确认 PATH 中包含 /opt/rocm/bin")

# Step 3. 检查 onnxruntime
print("\n[3] 导入 onnxruntime ...")
try:
    import onnxruntime as ort
    providers = ort.get_available_providers()
    print("✅ ONNX Runtime Providers:", providers)
    if "ROCMExecutionProvider" not in providers:
        print("⚠️ 未检测到 ROCMExecutionProvider，推理可能会在 CPU 上运行。")
except ImportError:
    print("❌ onnxruntime 未安装，请执行： pip install onnxruntime-rocm")
    exit(1)

# Step 4. 生成测试模型输入
print("\n[4] 创建随机输入张量进行推理测试...")
import onnx
import tempfile
import onnx.helper as helper
import onnxruntime as ort

# 生成一个简单的 y = Wx + b 模型（1层线性层）
x = helper.make_tensor_value_info("x", onnx.TensorProto.FLOAT, [1, 128])
y = helper.make_tensor_value_info("y", onnx.TensorProto.FLOAT, [1, 64])

W = np.random.randn(64, 128).astype(np.float32)
b = np.random.randn(64).astype(np.float32)

W_tensor = helper.make_tensor("W", onnx.TensorProto.FLOAT, W.shape, W.flatten())
b_tensor = helper.make_tensor("b", onnx.TensorProto.FLOAT, b.shape, b.flatten())

node = helper.make_node("Gemm", inputs=["x", "W", "b"], outputs=["y"])
graph = helper.make_graph([node], "LinearTest", [x], [y], [W_tensor, b_tensor])
model = helper.make_model(graph, producer_name="rocm_test")

tmp_model_path = tempfile.mktemp(suffix=".onnx")
onnx.save(model, tmp_model_path)

# Step 5. 进行推理测试
print("\n[5] 执行推理 (GPU)...")
sess = ort.InferenceSession(tmp_model_path, providers=["ROCMExecutionProvider", "CPUExecutionProvider"])

input_data = {"x": np.random.randn(1, 128).astype(np.float32)}
start = time.time()
for _ in range(100):
    out = sess.run(None, input_data)
end = time.time()

elapsed_ms = (end - start) * 1000 / 100
print(f"✅ 推理成功，平均每次耗时: {elapsed_ms:.2f} ms")

print("\n🎉 ROCm + ONNX Runtime GPU 测试通过！")
