import analysis

import matplotlib.pyplot as plt

import numpy as np

# - Compare cpu time over target count (for a given scenario) for each alg
def metrics_by_alg_percent(metric, specialized, metric_name, ylabel, header):

    algorithms = ['minimize_time', 'minimize_time_v2', 'static_greedy']

    algo_times = {
        '0.0': [],
        '0.1': [],
        '0.5': [],
        '0.8': [],
        '1.0': [],
    }

    for alg in algorithms:
        folders = analysis.filter_outputs({'dalg': alg, 'specialized': specialized})
        # print(alg)

        for f, info in folders:
            res = analysis.analyze_one(f)
            val = info['known_target_percentage']
            # print(val)
            v = res[metric]
            if metric == 'longest_path':
                v = v[1]
            elif metric == 'time_to_complete':
                v = v[0]
            algo_times[str(val)].append(v)

    # print(algo_times)

    x = np.arange(len(algorithms))  # the label locations
    width = 0.15  # the width of the bars
    multiplier = 0

    fig, ax = plt.subplots(layout='constrained')

    for attribute, measurement in algo_times.items():
        offset = width * multiplier
        rects = ax.bar(x + offset, measurement, width, label=attribute)
        ax.bar_label(rects, padding=3)
        multiplier += 1

    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_ylabel(metric_name + " " + ylabel)
    ax.set_title(f"{metric_name} {header}, {'with' if specialized else 'no'} specialization")
    ax.set_xticks(x + width, algorithms)
    ax.legend(loc='upper right', ncols=3)

    plt.savefig(f'figures/{metric}_{"spec" if specialized else "nospec"}.png')
    # plt.show()

def main():
    metrics = ['time_to_complete', 'longest_path', 'percent_moving']

    for metric in metrics:
        print(metric)
        for spec in [True, False]:
            metrics_by_alg_percent(metric, spec, "Path length", "(units)", "by Algorithm and % known")

if __name__ == '__main__':
    main()
