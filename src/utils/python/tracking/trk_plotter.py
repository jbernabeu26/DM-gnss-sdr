import mat73
import numpy as np
import matplotlib.pyplot as plt
import os


class trk_plotter:
    def __init__(self) -> None: 
        pass

    def setup(self, files):

        self.nsrc = len(files)
        self.outdir = {idx:'' for idx in range(self.nsrc)}
        self.data_dict = {i:0 for i in range(self.nsrc)}
        for idx, file in enumerate(files):
            self.outdir[idx] = os.path.dirname(file) 
            self.data_dict[idx] = mat73.loadmat(file)
    
    def plot_doppler(self, saveplot=True):
        ''' Compare tracking results from different channels'''
        # Plot doppler frequency
        fig, nax = plt.subplots(self.nsrc,1)
        for idx, ax in enumerate(nax):
            key = 'carrier_doppler_hz'
            ax.plot(self.data_dict[idx][key])
            ax.set_title('Carrier Doppler for PRN {}'.format(*set(self.data_dict[idx]['PRN'])))
        # set the spacing between subplots
        plt.subplots_adjust(left=0.1,
                            bottom=0.1, 
                            right=0.9, 
                            top=0.9, 
                            wspace=0.4, 
                            hspace=0.4)
        if saveplot:
            plt.savefig(os.path.join(self.outdir[0],'doppler.png'))

    def plot_IQPrompts(self, saveplot=True):
        ''' Plot I/Q prompts for different channels'''
        # Plot doppler frequency
        for idx in range(self.nsrc):
            fig, ax = plt.subplots()
            x = self.data_dict[idx]['Prompt_Q']
            y = self.data_dict[idx]['Prompt_I']
            ax.scatter(x, y)
            ax.set_title('I/Q Prompts for PRN {}'.format(*set(self.data_dict[idx]['PRN'])))
            if saveplot:
                _path = os.path.join(self.outdir[idx], 'IQ_PRN{}.png'.format(*set(self.data_dict[idx]['PRN'])))
                plt.savefig(_path)





