import torch
import torchvision.models as models
from torchvision import transforms
from PIL import Image
import time
from facenet_pytorch import InceptionResnetV1

# resnet = models.resnet18(pretrained=True)
resnet= InceptionResnetV1(pretrained='vggface2')

resnet.eval()
for param in resnet.parameters():
    param.requires_grad = False


transform = transforms.Compose([
    transforms.Resize(256),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])


start = time.time()
f1 = "face_0.jpg"
image1 = Image.open(f"/home/nac/Documents/projects/RINS/ROS/src/task1/images/{f1}")
image1 = transform(image1).unsqueeze(0)

f2 = "face_region.jpg"
image2 = Image.open(f"/home/nac/Documents/projects/RINS/ROS/src/task1/images/{f2}")
image2 = transform(image2).unsqueeze(0)

with torch.no_grad():
    feature1 = resnet(image1)
    feature2 = resnet(image2)

similarity = torch.nn.functional.cosine_similarity(feature1, feature2)
end = time.time()
print(f"similarity: {similarity},\ntime: {end-start}")
if similarity < 0.5:
    print(f"similarity smaller than 0.5: {similarity < 0.5}")
