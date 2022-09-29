from acq_plotter import acq_plotter
import os


def main():
    _absPath = './acquisition'
    subdir = '/data2'
    files = [file for file in os.listdir(_absPath + subdir) if '.mat' in file]
    plotter = acq_plotter()

    for file in files:
        plotter.load_file(_absPath + subdir + '/' + file)
        plotter.set_data()
        plotter.plot_data_2D()



if __name__ == '__main__':
    main()