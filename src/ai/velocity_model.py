import numpy as np
from sklearn.neural_network import MLPRegressor
import joblib

class VelocityModel:
    """
    Lightweight MLP temporal model for predicting forward velocity from IMU.
    Input Shape: (batch, window_size * 6)
    Output Shape: (batch, 1)
    """
    def __init__(self, window_size=20):
        self.window_size = window_size
        # Two hidden layers to act similarly to our 1D CNN + MLP
        self.model = MLPRegressor(
            hidden_layer_sizes=(128, 64),
            activation='relu',
            solver='adam',
            max_iter=50,
            early_stopping=True,
            validation_fraction=0.1,
            random_state=42
        )
        self.is_trained = False
        
    def fit(self, X: np.ndarray, y: np.ndarray):
        # Flatten the temporal windows for MLP
        X_flat = X.reshape(X.shape[0], -1)
        if X_flat.shape[0] < 20:
            self.model.set_params(early_stopping=False)
        else:
            self.model.set_params(early_stopping=True)
        self.model.fit(X_flat, y.ravel())
        self.is_trained = True
        
    def predict(self, X: np.ndarray) -> np.ndarray:
        if not self.is_trained:
            raise ValueError("Model is not trained yet!")
        X_flat = X.reshape(X.shape[0], -1)
        return self.model.predict(X_flat)
        
    def save(self, filepath: str):
        joblib.dump(self.model, filepath)
        
    def load(self, filepath: str):
        self.model = joblib.load(filepath)
        self.is_trained = True
