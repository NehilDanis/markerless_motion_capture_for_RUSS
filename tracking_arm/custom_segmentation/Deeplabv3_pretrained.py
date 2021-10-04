""" DeepLabv3 Model download and change the head for your prediction"""
from torchvision.models.segmentation.deeplabv3 import DeepLabHead
from torchvision import models
import segmentation_models_pytorch as smp


def createDeepLabv3(outputchannels=1):
    """DeepLabv3 class with custom head
    Args:
        outputchannels (int, optional): The number of output channels
        in your dataset masks. Defaults to 1.
    Returns:
        model: Returns the DeepLabv3 model with the ResNet101 backbone.
    """
    '''model = models.segmentation.deeplabv3_resnet50(pretrained=True,
                                                    progress=True)


    model.classifier = DeepLabHead(512, outputchannels)'''

    model = smp.Unet("vgg16", encoder_weights="imagenet", classes=1, activation=None)

    # Set the model in training mode
    #model.train()
    return model