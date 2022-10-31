import argparse
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import re


def main():
    """
    Parser for csv file export of PCM microphone data from Tracealyser.
    This was created for analysing the noisefloor from the microphone (see https://sitehive.atlassian.net/wiki/spaces/GAR/pages/1246527522/Microphone+signal)
    """
    parser = argparse.ArgumentParser(description='Analyse PCM csv file.')
    parser.add_argument('file', type=argparse.FileType('r'), help='PCM data')
    parser.add_argument("--start", action="store", type=int, default=32, help='Start index for plot')
    parser.add_argument("--end", action="store", type=int, default=-32, help='End index for plot')

    options = parser.parse_args()

    # Read the CVS with Panda
    df = pd.read_csv(options.file, delimiter=',', engine='python', header=1, skipfooter=1, names=['timestamp', 'actor', 'event_text'])

    # Extract the pcm data
    pattern = re.compile(r'\[noise_db\] DFSDM: (.*)')

    pcm_data = [int(pattern.match(x).groups(0)[0]) for x in df['event_text'].to_list()]

    # Plot the data
    fig, ax1 = plt.subplots(1, 1)

    rms = np.sqrt(np.mean((pcm_data[options.start:options.end] - np.mean(pcm_data[options.start:options.end])) ** 2))
    ax1.plot([x for x in pcm_data][options.start:options.end], 'bo-')
    ax1.title.set_text(f'{options.file.name}, rms: {rms}')

    print(f'rms: {rms}')

    plt.show()


if __name__ == '__main__':
    main()
