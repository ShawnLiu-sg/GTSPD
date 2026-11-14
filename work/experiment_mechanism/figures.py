import csv
from pathlib import Path
from reportlab.pdfgen.canvas import Canvas
from reportlab.lib import colors
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont


OUT = Path(__file__).resolve().parents[2] / 'outputs' / 'experiment-results' / 'mechanism'
pdfmetrics.registerFont(TTFont('Arial', r'C:\Windows\Fonts\arial.ttf'))
pdfmetrics.registerFont(TTFont('Arial Bold', r'C:\Windows\Fonts\arialbd.ttf'))
TEXT = colors.HexColor('#243243')
BLUE = colors.HexColor('#1B5A7C')
AMBER = colors.HexColor('#BA6A31')
GRID = colors.HexColor('#DAE1E7')
GRAY = colors.HexColor('#607080')


def read(name):
    with (OUT / name).open(newline='', encoding='utf-8') as source:
        return list(csv.DictReader(source))


def label(canvas, message, x, y, size=9, *, bold=False, color=TEXT, align='left'):
    canvas.setFont('Arial Bold' if bold else 'Arial', size)
    canvas.setFillColor(color)
    {'left': canvas.drawString, 'right': canvas.drawRightString, 'center': canvas.drawCentredString}[align](x, y, message)


def dot(canvas, x, y, color, radius=2.6, open_dot=False):
    canvas.setStrokeColor(color)
    canvas.setFillColor(colors.white if open_dot else color)
    canvas.setLineWidth(1)
    canvas.circle(x, y, radius, fill=1, stroke=1)


def title(canvas, main, subtitle, height):
    label(canvas, main, 20, height - 23, 15, bold=True)
    label(canvas, subtitle, 20, height - 40, 9, color=GRAY)


def operators():
    names = {'random':'Random', 'spatial_relatedness':'Spatial relatedness', 'worst_nonlaunch':'Worst non-launch',
             'worst_launch':'Worst launch', 'edge_based':'Edge-based', 'longest_sortie':'Longest sortie',
             'greedy':'Greedy', 'regret_2':'Regret-2', 'random_greedy':'Random greedy', 'off':'Off', 'on':'On'}
    rows = {row['operator']: row for row in read('operator_summary.csv') if row['size'] == 'all'}
    path = OUT / 'figure2_operator_diagnostics.pdf'
    c = Canvas(str(path), pagesize=(468, 370))
    title(c, 'ALNS operator diagnostics', '60 instances  •  19,100 completed iterations  •  1.5 s per instance', 370)
    plot_left, plot_right = 139, 414
    bottom, top = 64, 295
    for value in range(0, 41, 10):
        x = plot_left + (plot_right - plot_left) * value / 40
        c.setStrokeColor(GRID)
        c.setLineWidth(0.7)
        c.line(x, bottom, x, top)
        label(c, str(value), x, bottom - 13, 8, align='center')
    y = 292
    for category, group in (
            ('Destroy', ('random','spatial_relatedness','worst_nonlaunch','worst_launch','edge_based','longest_sortie')),
            ('Repair', ('greedy','regret_2','random_greedy')), ('Noise', ('off','on'))):
        label(c, category, 20, y, 9, bold=True)
        y -= 15
        for name in group:
            row = rows[name]
            label(c, names[name], 129, y - 2, 8, align='right')
            rates = [float(row[k]) for k in ('improved_best_pct_of_completed','improved_current_pct_of_completed')]
            p1, p2 = [plot_left + (plot_right - plot_left) * x / 40 for x in rates]
            c.setStrokeColor(GRID)
            c.line(p1, y, p2, y)
            dot(c, p1, y, BLUE, open_dot=True)
            dot(c, p2, y, AMBER)
            label(c, f"{int(row['completed']):,}", 448, y - 2, 8, align='right', color=GRAY)
            y -= 14
        y -= 4
    label(c, 'n', 448, top, 8, align='right', color=GRAY)
    label(c, 'Improvement rate per completed invocation (%)', (plot_left + plot_right) / 2, 39, 8, align='center')
    dot(c, 140, 20, BLUE, open_dot=True)
    label(c, 'Best-so-far solution', 149, 17, 8)
    dot(c, 250, 20, AMBER)
    label(c, 'Current solution', 259, 17, 8)
    c.showPage()
    c.save()


def neighborhood():
    rows = read('neighborhood_replay.csv')
    path = OUT / 'figure_neighborhood_replay.pdf'
    c = Canvas(str(path), pagesize=(468, 294))
    title(c, 'Value of cross-stop reassignment', '60 fixed-truck-route snapshots  •  Costs checked by exhaustive enumeration', 294)
    left, right, bottom, top = 50, 449, 47, 229
    maximum = 2.5
    for i in range(6):
        v = i / 2
        y = bottom + (top - bottom) * v / maximum
        c.setStrokeColor(GRID)
        c.setLineWidth(0.75)
        c.line(left, y, right, y)
        label(c, f'{v:.1f}', left - 7, y - 3, 8, align='right')
    for column, (n, geom) in enumerate((n, geo) for n in ('50', '100') for geo in ('U', 'R', 'CL')):
        group = [r for r in rows if r['size'] == n and r['geometry'] == geom]
        center = left + (column + 0.5) * (right - left) / 6
        label(c, {'U':'Uniform', 'R':'Ring', 'CL':'Clustered'}[geom], center, bottom - 14, 8, align='center')
        for i, row in enumerate(group):
            val = float(row['within_stop_cny']) - float(row['cross_stop_cny'])
            x = center + (i - 4.5) * 4.6
            y = bottom + (top - bottom) * val / maximum
            dot(c, x, y, BLUE if n == '50' else AMBER, 2.0, open_dot=val <= 1e-8)
    c.setStrokeColor(GRID)
    c.line((left + right) / 2, bottom - 4, (left + right) / 2, top)
    label(c, '50 customers', left + (right - left) / 4, 17, 9, align='center', bold=True)
    label(c, '100 customers', left + 3 * (right - left) / 4, 17, 9, align='center', bold=True)
    c.saveState()
    c.translate(15, (bottom + top) / 2)
    c.rotate(90)
    label(c, 'Cost saved beyond within-stop reoptimization (CNY)', 0, 0, 8, align='center')
    c.restoreState()
    c.showPage()
    c.save()


operators()
neighborhood()
