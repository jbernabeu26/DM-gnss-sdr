from trk_plotter import trk_plotter
import os


def main():
    _Path = './tracking/data2'
    files = [os.path.join(_Path, file) for file in os.listdir(_Path) if '.mat' in file]
    plotter = trk_plotter()
    # Load tracking results from N files into a dictonary
    plotter.setup(files)
    plotter.plot_IQPrompts()
    plotter.plot_doppler()




if __name__ == '__main__':
    main()