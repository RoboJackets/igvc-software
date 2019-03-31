import argparse
import cv2
import numpy as np
import os
import pdb
import torch
import torch.optim as optim
from torch.autograd import Variable
import torch.nn.functional as F
from torchvision import transforms

from IGVCDataset import IGVCDataset

import models.model
import utils

np.set_printoptions(threshold=np.nan)
torch.set_printoptions(precision=10)


# Training settings.
parser = argparse.ArgumentParser(description='IGVC segmentation of lines.')

# Hyperparameters.
parser.add_argument('--batch_size', type=int, default=1,
                    help='input batch size for training.')
parser.add_argument('--epochs', type=int, default=5,
                    help='number of epochs to train')
parser.add_argument('--im_size', type=int, nargs=3, default=[3,400,400],
                    help='image dimensions for training.')
parser.add_argument('--kernel_size', type=int, default=3,
                    help='size of convolution kernels/filters.')
parser.add_argument('--lr', type=float, default=1e-3,
                    help='learning rate.')
parser.add_argument('--lr_decay', type=float, default=1.0,
                    help='Learning rate decay multiplier.')
parser.add_argument('--step_interval', type=int, default=100,
                    help='Update learning rate every <step_interval> epochs.')
parser.add_argument('--weight_decay', type=float, default=0.0,
                    help='Weight decay hyperparameter.')

# Other configuration.
parser.add_argument('--save_model', action='store_true', default=False,
                    help='Save pytorch model.')
parser.add_argument('--save_interval', type=int, default=1,
                    help='Save pytorch model after <save_interval> epochs.')
parser.add_argument('--load_model', type=str, default=None,
                    help='Load model from .pt file, either for initialization or evaluation.')
parser.add_argument('--log_interval', type=int, default=10,
                    help='number of batches between logging train status.')
parser.add_argument('--vis', action='store_true', default=False,
                    help='Visualize model output every log interval.')
parser.add_argument('--no_cuda', action='store_true', default=False,
                    help='disables CUDA training')
parser.add_argument('--seed', type=int, default=1, metavar='S',
                    help='random seed (default: 1)')
parser.add_argument('--cfgfile', type=str, default='cfg/igvc.cfg',
                    help='Directory containing cfg for train and evaluation.')
parser.add_argument('--test', action='store_true', default=False,
                    help='Skip training, and evaluate a loaded model on test data.')
parser.add_argument('--val_samples', type=int, default=10,
                    help='Number of validation samples to use from train data.')

args = parser.parse_args()

args.cuda = not args.no_cuda and torch.cuda.is_available()
torch.manual_seed(args.seed)
if args.cuda:
    print('Using cuda.')
    torch.cuda.manual_seed(args.seed)

kwargs = {'num_workers': 1, 'pin_memory': True} if args.cuda else {}

transform = transforms.Compose([
                 transforms.ToTensor(),
            ])

cfg_params = utils.read_data_cfg(args.cfgfile)

train_txt = cfg_params['train']
test_txt = cfg_params['test']
backup_dir = cfg_params['backup']

if args.load_model is not None:
    print('Loading model from %s.' % args.load_model)
    model = models.model.UNet(args.im_size, args.kernel_size)
    model.load_state_dict(torch.load(args.load_model))
elif args.test:
    print('Missing model file for evaluating test set.')
    exit()
else:
    model = models.model.UNet(args.im_size, args.kernel_size)

# Datasets and dataloaders.
if not args.test:
    train_dataset = IGVCDataset(train_txt, im_size=args.im_size, split='train', transform=transform, val_samples=args.val_samples)
    val_dataset = IGVCDataset(train_txt, im_size=args.im_size, split='val', transform=transform, val_samples=args.val_samples)
    train_loader = torch.utils.data.DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True, **kwargs)
    val_loader = torch.utils.data.DataLoader(val_dataset, batch_size=args.batch_size, shuffle=True, **kwargs)
    
    # Optmizer
    lr = args.lr
    print('Initial lr: %f.' % lr)
    optimizer = optim.Adam(model.parameters(), lr=lr, weight_decay=args.weight_decay)
else:
    test_dataset = IGVCDataset(test_txt, im_size=args.im_size, split='test', transform=transform)
    test_loader = torch.utils.data.DataLoader(test_dataset, batch_size=1, shuffle=True, **kwargs)

criterion = F.binary_cross_entropy
if args.cuda:
    model.cuda()

def train(epoch):
    iters = []
    lrs = []
    train_losses = []
    val_losses = []
    val_accuracies = []
    model.train()
    # train loop
    for batch_idx, batch in enumerate(train_loader):
        # prepare data
        images = Variable(batch[0])
        targets = Variable(batch[1])

        if args.cuda:
            images, targets = images.cuda(), targets.cuda()

        optimizer.zero_grad()
        outputs = model(images)
        loss = criterion(outputs, targets)
        loss.backward()
        optimizer.step()

        if args.vis and batch_idx % args.log_interval == 0 and images.shape[0] == 1:
            cv2.imshow('output: ', outputs.cpu().data.numpy()[0][0])
            cv2.imshow('target: ', targets.cpu().data.numpy()[0][0])
            cv2.waitKey(10)

        # Learning rate decay.
        if epoch % args.step_interval == 0 and epoch != 1 and batch_idx == 0:
            if args.lr_decay != 1:
                global lr, optimizer
                lr *= args.lr_decay
                for param_group in optimizer.param_groups:
                    param_group['lr'] = lr
                print('Learning rate decayed to %f.' % lr)

        if batch_idx % args.log_interval == 0:
            val_loss, val_acc = evaluate('val', n_batches=80)
            train_loss = loss.item()
            iters.append(len(train_loader.dataset)*(epoch-1)+batch_idx)
            lrs.append(lr)
            train_losses.append(train_loss)
            val_losses.append(val_loss)
            val_accuracies.append(val_acc)

            examples_this_epoch = batch_idx * len(images)
            epoch_progress = 100. * batch_idx / len(train_loader)
            print('Train Epoch: {} [{}/{} ({:.0f}%)]\t'
                  'Train Loss: {:.6f}\tVal Loss: {:.6f}\tVal Acc: {}'.format(
                epoch, examples_this_epoch, len(train_loader.dataset),
                epoch_progress, train_loss, val_loss, val_acc))

    return iters, train_losses, val_losses, val_accuracies

def evaluate(split, verbose=False, n_batches=None):
    '''
    Compute loss on val or test data.
    '''
    model.eval()
    loss = 0
    acc = 0
    correct = 0
    n_examples = 0
    if split == 'val':
        loader = val_loader
    elif split == 'test':
        loader = test_loader
    for batch_i, batch in enumerate(loader):
        data, target = batch
        if args.cuda:
            data, target = data.cuda(), target.cuda()
        data, target = Variable(data, volatile=True), Variable(target)
        output = model(data)
        loss += criterion(output, target).item()
        acc += (np.sum(output.cpu().data.numpy()[target.cpu().data.numpy()!=0] > 0.5) \
            + np.sum(output.cpu().data.numpy()[target.cpu().data.numpy()==0] < 0.5)) / float(args.im_size[1]*args.im_size[2])
        n_examples += output.size(0)

        if n_batches and (batch_i == n_batches-1):
            break

    loss /= n_examples
    acc /= n_examples
    return loss, acc

if args.test:
    print("Running evaluation on test set.")
    test_loss, test_acc = evaluate('test')
    print('Test loss: %f  Test accuracy: %f' % (test_loss, test_acc))
else:
    # train the model one epoch at a time
    metrics = {'iters':[], 'train_loss':[], 'val_loss':[], 'val_acc':[]}
    for epoch in range(1, args.epochs + 1):
        iters, train_losses, val_losses, val_accuracies = train(epoch)
        metrics['iters'] += iters
        metrics['train_loss'] += train_losses
        metrics['val_loss'] += val_losses
        metrics['val_acc'] += val_accuracies
        if (epoch % args.save_interval == 0 and args.save_model):
            save_path = os.path.join(backup_dir, 'IGVCModel' + '_' + str(epoch) + '.pt')
            print('Saving model: %s' % save_path)
            torch.save(model.state_dict(), save_path)
        metrics_path = os.path.join(backup_dir, 'metrics.npy')
        np.save(metrics_path, metrics)

