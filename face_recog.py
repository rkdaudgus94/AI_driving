import os
import pandas as pd
import numpy as np
import cv2
from tqdm import tqdm_notebook as tqdm
from matplotlib import pyplot as plt
from PIL import Image
import torch
import time

video_path = '../input/train_sample_videos/'

reader = cv2.VideoCapture(video_path + 'abqwwspghj.mp4')

image_list = []
for i in tqdm(range(int(reader.get(cv2.CAP_PROP_FRAME_COUNT)))):
    _, image = reader.read()
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_list.append(image)
reader.release()

images_np = np.stack(image_list)

images_np.shape

def plot_faces(images, figsize=(10.8/2, 19.2/2)):
    shape = images[0].shape
    images = images[np.linspace(0, len(images)-1, 16).astype(int)]
    im_plot = []
    for i in range(0, 16, 4):
        im_plot.append(np.concatenate(images[i:i+4], axis=0))
    im_plot = np.concatenate(im_plot, axis=1)
    
    fig, ax = plt.subplots(1, 1, figsize=figsize)
    ax.imshow(im_plot)
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False)

    ax.grid(False)
    fig.tight_layout()

def timer(detector, detect_fn, images, *args):
    start = time.time()
    faces = detect_fn(detector, images, *args)
    elapsed = time.time() - start
    print(f', {elapsed:.3f} seconds')
    return faces, elapsed


device = 'cuda:1' if torch.cuda.is_available() else 'cpu'

from facenet_pytorch import MTCNN
detector = MTCNN(device=device, post_process=False)

def detect_facenet_pytorch(detector, images, batch_size):
    faces = []
    for lb in np.arange(0, len(images), batch_size):
        imgs_pil = [Image.fromarray(image) for image in images[lb:lb+batch_size]]
        faces.extend(detector(imgs_pil))
    return faces

times_facenet_pytorch = []    # batched
times_facenet_pytorch_nb = [] # non-batched

print('Detecting faces in 1080x1920 frames', end='')
faces, elapsed = timer(detector, detect_facenet_pytorch, image_list, 20)
times_facenet_pytorch.append(elapsed)

plot_faces(torch.stack(faces).permute(0, 2, 3, 1).int().numpy())

del detector
torch.cuda.empty_cache()