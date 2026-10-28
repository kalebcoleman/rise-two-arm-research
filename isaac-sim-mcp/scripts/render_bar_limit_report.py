"""Render the measured load/speed outcomes without interpolating untested conditions."""

import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def render(output_directory):
    """Save PNG/SVG maps of the observed trial outcomes and repeated conditions."""
    output = Path(output_directory)
    report = json.loads((output / 'summary.json').read_text())
    masses = sorted({t['mass_kg'] for t in report['trials']})
    speeds = sorted({t['requested_speed_scale'] for t in report['trials']})
    groups = {}
    for trial in report['trials']:
        groups.setdefault((trial['mass_kg'], trial['requested_speed_scale']), []).append(trial)
    styles = {'Pass': '#b6e2c5', 'Drift': '#ffe0a3', 'Drop': '#f3b3af',
              'Track': '#d8caef', 'Mixed': '#cccccc'}
    figure, ax = plt.subplots(figsize=(9, 6.5))
    for row, mass in enumerate(masses):
        for column, speed in enumerate(speeds):
            trials = groups.get((mass, speed), [])
            categories = set()
            for trial in trials:
                if trial['pass']:
                    categories.add('Pass')
                elif 'drift' in trial.get('failure_class', ''):
                    categories.add('Drift')
                elif 'tracking' in trial.get('failure_class', ''):
                    categories.add('Track')
                else:
                    categories.add('Drop')
            label = next(iter(categories)) if len(categories) == 1 else ('Mixed' if categories else '')
            ax.add_patch(Rectangle((column-.48, row-.48), .96, .96,
                                  facecolor=styles.get(label, '#f2f3f5'), edgecolor='white'))
            text = label if len(trials) < 2 else f'{label}\n({len(trials)} trials)'
            if label:
                ax.text(column, row, text, ha='center', va='center', fontsize=10)
    ax.set(xlim=(-.5, len(speeds)-.5), ylim=(-.5, len(masses)-.5),
           xticks=range(len(speeds)), yticks=range(len(masses)),
           xticklabels=[f'{x:g}x' for x in speeds], yticklabels=[f'{x:g}' for x in masses],
           xlabel='Requested trajectory speed scale (shorter segment times)', ylabel='Bar mass (kg)')
    ax.set_title('Observed two-gripper load/speed outcomes', loc='left', fontsize=15, fontweight='bold', pad=30)
    ax.text(0, 1.025, f'{len(report["trials"])} simulated trials; blank cells untested; object geometry/friction and drive caps fixed',
            transform=ax.transAxes, fontsize=9, color='#444444')
    figure.text(.12, .035,
                'Drift: >2 mm relative grasp motion. Track: >10 mm object-goal error. Drop: bar reached table.\n'
                'These are tested simulation brackets, not physical RG2 ratings or a general safe envelope.',
                fontsize=9, color='#333333')
    ax.spines[['top', 'right', 'left', 'bottom']].set_visible(False)
    ax.tick_params(length=0, pad=8)
    figure.subplots_adjust(left=.12, right=.97, top=.82, bottom=.17)
    figure.savefig(output / 'operating-map.png', dpi=180, facecolor='white')
    figure.savefig(output / 'operating-map.svg', facecolor='white')
    plt.close(figure)
