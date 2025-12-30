"""
Stewart平台正运动学 - 神经网络求解
功能：通过监督学习，从杆长预测位姿
"""

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
from tqdm import tqdm

# 设置随机种子
torch.manual_seed(42)
np.random.seed(42)

# 设置设备
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"使用设备: {device}")


# ============== 神经网络模型定义 ==============
class ForwardKinematicsNN(nn.Module):
    """正运动学神经网络：输入6个杆长，输出6个位姿参数"""

    def __init__(self):
        super(ForwardKinematicsNN, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(6, 128),
            nn.ReLU(),
            nn.Linear(128, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, 6)
        )

    def forward(self, x):
        return self.network(x)


# ============== 数据加载与预处理 ==============
def load_data(filepath='training_data.csv'):
    """加载MATLAB生成的训练数据"""
    print(f"正在加载数据: {filepath}")
    data = pd.read_csv(filepath)

    # 输入：6个杆长 (L1-L6)
    X = data[['L1', 'L2', 'L3', 'L4', 'L5', 'L6']].values

    # 输出：6个位姿参数 (x,y,z,roll,pitch,yaw)
    y = data[['x', 'y', 'z', 'roll', 'pitch', 'yaw']].values

    print(f"数据加载完成: {X.shape[0]} 样本")
    return X, y


# ============== 训练函数（带tqdm进度条） ==============
def train_model(model, train_loader, val_loader, epochs=200, lr=0.001):
    """训练神经网络"""
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode='min', factor=0.5, patience=10
    )

    train_losses = []
    val_losses = []

    # 使用tqdm显示epoch进度
    pbar = tqdm(range(epochs), desc="训练进度")
    for epoch in pbar:
        # 训练阶段
        model.train()
        train_loss = 0
        for X_batch, y_batch in train_loader:
            X_batch, y_batch = X_batch.to(device), y_batch.to(device)
            optimizer.zero_grad()
            outputs = model(X_batch)
            loss = criterion(outputs, y_batch)
            loss.backward()
            optimizer.step()
            train_loss += loss.item()

        train_loss /= len(train_loader)
        train_losses.append(train_loss)

        # 验证阶段
        model.eval()
        val_loss = 0
        with torch.no_grad():
            for X_batch, y_batch in val_loader:
                X_batch, y_batch = X_batch.to(device), y_batch.to(device)
                outputs = model(X_batch)
                loss = criterion(outputs, y_batch)
                val_loss += loss.item()

        val_loss /= len(val_loader)
        val_losses.append(val_loss)
        scheduler.step(val_loss)

        # 更新进度条显示
        pbar.set_postfix({
            'Train Loss': f'{train_loss:.6f}',
            'Val Loss': f'{val_loss:.6f}'
        })

    return train_losses, val_losses


# ============== 主函数 ==============
if __name__ == '__main__':
    # 1. 加载数据
    X, y = load_data('training_data.csv')

    # 2. 数据标准化
    scaler_X = StandardScaler()
    scaler_y = StandardScaler()
    X_scaled = scaler_X.fit_transform(X)
    y_scaled = scaler_y.fit_transform(y)

    # 3. 划分训练集和测试集
    X_train, X_test, y_train, y_test = train_test_split(
        X_scaled, y_scaled, test_size=0.2, random_state=42
    )

    # 4. 转换为PyTorch张量
    X_train_t = torch.FloatTensor(X_train)
    y_train_t = torch.FloatTensor(y_train)
    X_test_t = torch.FloatTensor(X_test)
    y_test_t = torch.FloatTensor(y_test)

    # 5. 创建DataLoader
    train_dataset = TensorDataset(X_train_t, y_train_t)
    test_dataset = TensorDataset(X_test_t, y_test_t)
    train_loader = DataLoader(train_dataset, batch_size=64, shuffle=True)
    test_loader = DataLoader(test_dataset, batch_size=64)

    # 6. 创建模型并训练
    print("\n开始训练神经网络...")
    model = ForwardKinematicsNN().to(device)
    train_losses, val_losses = train_model(
        model, train_loader, test_loader, epochs=200, lr=0.001
    )

    # 7. 测试模型
    print("\n========== 模型评估 ==========")
    model.eval()
    with torch.no_grad():
        X_test_device = X_test_t.to(device)
        y_pred_scaled = model(X_test_device).cpu().numpy()
        y_pred = scaler_y.inverse_transform(y_pred_scaled)
        y_true = scaler_y.inverse_transform(y_test)

    # 计算各维度误差
    errors = np.abs(y_pred - y_true)
    labels = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
    print("\n各维度平均绝对误差:")
    for i, label in enumerate(labels):
        print(f"  {label}: {errors[:, i].mean():.4f}")

    # 8. 绘制训练曲线
    plt.figure(figsize=(10, 4))
    plt.subplot(1, 2, 1)
    plt.plot(train_losses, label='Train Loss')
    plt.plot(val_losses, label='Val Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.title('Training Curve')
    plt.legend()
    plt.grid(True)

    # 预测vs真实对比图
    plt.subplot(1, 2, 2)
    plt.scatter(y_true[:500, 0], y_pred[:500, 0], alpha=0.5, s=10)
    plt.plot([-10, 10], [-10, 10], 'r--')
    plt.xlabel('True x')
    plt.ylabel('Predicted x')
    plt.title('Prediction vs True (x)')
    plt.grid(True)

    plt.tight_layout()
    plt.savefig('training_results.png', dpi=150)
    plt.show()
    print("\n训练结果已保存到 training_results.png")

    # 9. 保存模型
    torch.save(model.state_dict(), 'forward_kinematics_model.pth')
    print("模型已保存到 forward_kinematics_model.pth")
