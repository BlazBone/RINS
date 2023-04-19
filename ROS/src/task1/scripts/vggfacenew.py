import torch
import pretrainedmodels
from PIL import Image
from torchvision import transforms

# Load the pre-trained VGGFace2 model
vgg_model = pretrainedmodels.models.vggface2(pretrained='vggface2')

# Define a preprocessing pipeline for input images
preprocess = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize(mean=vgg_model.mean, std=vgg_model.std)
])

# Load two input images
img1 = Image.open('path/to/image1.jpg')
img2 = Image.open('path/to/image2.jpg')

# Preprocess the images
img1_tensor = preprocess(img1)
img2_tensor = preprocess(img2)

# Extract features from the images
with torch.no_grad():
    img1_features = vgg_model.features(torch.unsqueeze(img1_tensor, 0)).squeeze()
    img2_features = vgg_model.features(torch.unsqueeze(img2_tensor, 0)).squeeze()

# Compute the cosine similarity between the image features
cos_sim = torch.nn.functional.cosine_similarity(img1_features, img2_features)
print(f"The cosine similarity between the two images is {cos_sim.item()}")

