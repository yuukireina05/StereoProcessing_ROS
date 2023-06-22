from tensorflow.keras.models import load_model

from keras import backend as keras


class seg_model():
    def __init__(self, model_PATH):
        self.model = load_model(model_PATH, custom_objects={'dice_coef': self.dice_coef, 'jacard': self.jacard})

    def getModel(self):
        return self.model

    def predict(self, img):
        pred = self.model.predict(img, verbose=1)
        return pred

    def dice_coef(self,y_true, y_pred):
        smooth = 0.0
        y_true_f = keras.flatten(y_true)
        y_pred_f = keras.flatten(y_pred)
        intersection = keras.sum(y_true_f * y_pred_f)
        return (2. * intersection + smooth) / (keras.sum(y_true_f) + keras.sum(y_pred_f) + smooth)

    def jacard(self, y_true, y_pred):
        y_true_f = keras.flatten(y_true)
        y_pred_f = keras.flatten(y_pred)
        intersection = keras.sum(y_true_f * y_pred_f)
        union = keras.sum(y_true_f + y_pred_f - y_true_f * y_pred_f)

        return intersection / union