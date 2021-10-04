import torch
import matplotlib.pyplot as plt
import cv2
import pandas as pd
from torchvision import transforms
from PIL import Image


# Load the trained model
model = torch.load('/home/nehil/new_arm_w_tarso_data_folder/weights_vgg16_2.pt')
# Set the model to evaluate mode
model.eval()


# Read the log file using pandas into a dataframe
df = pd.read_csv('/home/nehil/new_arm_w_tarso_data_folder/log_vgg16.csv')

# Plot all the values with respect to the epochs
#df.plot(x='epoch',figsize=(15,8));



#print(df[['Train_auroc','Test_auroc']].max())


# Read  a sample image and mask from the data-set

transform = transforms.Compose([transforms.Resize((512,512)), transforms.ToTensor()])
img = cv2.imread(f'/home/nehil/images_arm/img276.jpg')

img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img = cv2.resize(img, (512, 512)).transpose(2,0,1).reshape(1,3,512,512)
print(img.shape)

with torch.no_grad():
    a = model(torch.from_numpy(img).type(torch.cuda.FloatTensor) / 255)


# Plot histogram of the prediction to find a suitable threshold. From the histogram a 0.1 looks like a good choice.
plt.hist(a.data.cpu().numpy().flatten())


# Plot the input image, ground truth and the predicted output
plt.figure(figsize=(10,10));
plt.subplot(131);
plt.imshow(img[0,...].transpose(1,2,0));
plt.title('Image')
plt.axis('off');
plt.subplot(133);
plt.imshow(a.cpu().detach().numpy()[0][0]>0.2);
plt.title('Segmentation Output')
plt.axis('off');
plt.savefig('/home/nehil/arm_w_tarso_data_folder/output.jpg',bbox_inches='tight')


