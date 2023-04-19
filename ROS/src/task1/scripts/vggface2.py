import numpy as np
from keras_vggface.vggface import VGGFace

model = VGGFace(model='resnet50', include_top=False, input_shape=(224, 224, 3), pooling='avg')

from keras_vggface.utils import preprocess_input
from PIL import Image

def preprocess_image(image_path):
    image = Image.open(image_path)
    image = image.resize((224, 224))
    image = np.array(image)
    image = np.expand_dims(image, axis=0)
    image = preprocess_input(image)
    return image


def extract_features(image_path):
    image = preprocess_image(image_path)
    features = model.predict(image)
    return features

from scipy.spatial.distance import cosine

def compare_images(image1_path, image2_path):
    features1 = extract_features(image1_path)
    features2 = extract_features(image2_path)
    similarity = 1 - cosine(features1, features2)
    return similarity
