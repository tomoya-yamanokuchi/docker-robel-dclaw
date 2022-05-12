import torch
import time

# Check version and device
print(torch.__version__)
print(torch.cuda.is_available())

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
m = torch.nn.Linear(20, 30).to(DEVICE)
input = torch.randn(128, 20).to(DEVICE)
output = m(input)
print('output', output.size())


# Test CPU
start_time = time.time()
a = torch.ones(4, 4)
for _ in range(1000000):
    a += a
elapsed_time = time.time() - start_time
print("CPU time = ", elapsed_time)


# Test GPU
start_time = time.time()
b = torch.ones(4, 4).cuda()
for _ in range(1000000):
    b += b
elapsed_time = time.time() - start_time
print("GPU time = ", elapsed_time)
