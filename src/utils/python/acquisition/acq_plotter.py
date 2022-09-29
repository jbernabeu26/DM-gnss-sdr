from ast import main
import mailcap
from tkinter.tix import MAIN
from unittest.main import MAIN_EXAMPLES
import mat73
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import os

class acq_plotter:
    def __init__(self) -> None: 
        pass

    def load_file(self, filepath):
        self.filepath = filepath
        self.data_dict = mat73.loadmat(filepath)

    def set_data(self):
        '''Format data for plotting'''
        self.zz = self.data_dict['acq_grid']
        # Get cdelay and carrier freq
        row_idx = np.argmax(self.zz, axis= 0)
        row_max = np.max(self.zz, axis=0)

        col_idx = np.argmax(row_max)

        cdelay_idx = row_idx[col_idx]

        self.fspace = self.zz[cdelay_idx, :]
        self.cspace = self.zz[:, col_idx]
    
        # 3D plane
        x = np.arange(np.shape(self.zz)[0])
        y = np.arange(np.shape(self.zz)[1])
        self.xx, self.yy = np.meshgrid(x, y)

    def plot_data_2D(self, savefig = True):
        # Carrier Freq space
        fig, ax = plt.subplots(2,1)
        doppler_max =  int(self.data_dict['doppler_max'])
        fstep =  int(self.data_dict['doppler_step'])
        xx = np.linspace(-doppler_max, doppler_max, int(doppler_max*2/fstep))
        ax[0].plot(xx, self.fspace)
        # code phase space
        xx = np.linspace(0, 10230, len(self.cspace))
        ax[1].plot(xx, self.cspace)
        cdelay_samples = self.data_dict['acq_delay_samples']
        print('Cdelay is: {} Samples, {} Chips'.format(int(cdelay_samples), int(cdelay_samples*10230/100e3)))
        for ax_id in range(2):
            ax[ax_id].set_xlabel(['Doppler [Hz]', 'Code Shift [Chips]'][ax_id])
            ax[ax_id].set_ylabel('Correlation')
        plt.subplots_adjust(hspace=0.4)    
        fig.suptitle('Acq Results for PRN {}'.format(int(self.data_dict['PRN'])))
        # Save figure if selected    
        if savefig:
            dirname = os.path.dirname(self.filepath)
            figname = os.path.join(dirname, os.path.basename(self.filepath).split('.')[0] + '.png')
            plt.savefig(figname)

    def plot_data_3D(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(self.xx, self.yy, self.zz.transpose(), cmap="plasma", linewidth=0, antialiased=False, alpha=0.5)


if __name__ == '__main__':

    filename = 'acq_data/bds_b1c_acq_C_C1_ch_0_1_sat_19.mat'

    plotter = acq_plotter()
    plotter.load_file(filename)
    plotter.set_data()
    plotter.plot_data_2D()

