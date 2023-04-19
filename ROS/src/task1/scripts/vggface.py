import torch
from facenet_pytorch import InceptionResnetV1

model = InceptionResnetV1(pretrained='vggface2')
model.eval()
from PIL import Image
import torchvision.transforms.functional as F

# Load and preprocess image
img1 = Image.open('/home/nac/Documents/projects/RINS/ROS/src/task1/images/face_1.jpg')
img1 = F.resize(img1, (160, 160))
img1 = F.to_tensor(img1).unsqueeze(0)

# Get facial embedding
# embedding1 = model(img1).detach().numpy()
embedding1 = model(img1).detach()

# Load and preprocess image
img2 = Image.open('/home/nac/Documents/projects/RINS/ROS/src/task1/images/face_0.jpg')
img2 = F.resize(img2, (160, 160))
img2 = F.to_tensor(img2).unsqueeze(0)

# Get facial embedding
# embedding2 = model(img1).detach().numpy()
embedding2 = model(img1).detach()
distance = torch.nn.functional.cosine_similarity(embedding1, embedding2)

print(distance)

