import os
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error
from src.data.loader import load_s_dataset
from src.ai.features import create_windows
from src.ai.velocity_model import VelocityModel

def train():
    print("Loading data for training...")
    data_path = r"IO-VNBD/Unsynchronised V and S Dataset/Uncategorised IOVNB (V and S) Dataset/S-Dataset/S-Vta10.csv"
    packets = load_s_dataset(data_path)
    
    print("Extracting IMU windows and GNSS speed targets...")
    X, Y = create_windows(packets, window_size=20)
    
    if len(X) == 0:
        print("No valid training pairs found!")
        return
        
    print(f"Generated {len(X)} training samples.")
    
    X_train, X_val, y_train, y_val = train_test_split(X, Y, test_size=0.2, random_state=42)
    
    model = VelocityModel(window_size=20)
    
    print("Starting training...")
    model.fit(X_train, y_train)
    
    train_preds = model.predict(X_train)
    val_preds = model.predict(X_val)
    
    train_mse = mean_squared_error(y_train, train_preds)
    val_mse = mean_squared_error(y_val, val_preds)
    
    print(f"Training finished | Train MSE: {train_mse:.4f} | Val MSE: {val_mse:.4f}")
            
    os.makedirs("src/models", exist_ok=True)
    model.save("src/models/velocity_model.pkl")
    print("Model saved to src/models/velocity_model.pkl")

if __name__ == "__main__":
    train()
