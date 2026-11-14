import argparse
import csv
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


root = Path(__file__).resolve().parents[1] / 'outputs' / 'experiment-results'
fonts = Path('C:/Windows/Fonts')
colors = {'hybrid':'#186547', 'plain':'#4E70AD', 'within_stop':'#9A3E88',
          'order_first':'#B66200','vns':'#54606A'}
labels = {'hybrid':'Hybrid ALNS', 'plain':'Plain ALNS', 'within_stop':'Within-stop ALNS',
          'order_first':'Order-first', 'vns':'VNS'}


def plot(directory, screen):
    with (directory/'figure3_checkpoints.csv').open(encoding='utf-8-sig',newline='') as source:
        rows = list(csv.DictReader(source))
    size = (2120, 980)
    image = Image.new('RGB',size,'white')
    draw = ImageDraw.Draw(image)
    face = lambda s: ImageFont.truetype(str(fonts/'arial.ttf'),s)
    bold = lambda s: ImageFont.truetype(str(fonts/'arialbd.ttf'),s)
    title = 'Short-budget algorithm screening' if screen else 'Anytime performance under matched wall-clock budgets'
    draw.text((114,62),title,fill='#172D3E',font=bold(55))
    draw.text((114,130),'Mean improvement relative to the all-truck starting solution (%)',fill='#253E4D',font=face(33))

    for idx,n in enumerate(('50','100')):
        data = [r for r in rows if r['n']==n and r['method'] in colors]
        if not data:
            continue
        left,top,right,bottom = 176+idx*1000,267,965+idx*1000,792
        maximum_ms = max(int(r['checkpoint_ms']) for r in data)
        ymin = 0
        ymax = 5 * (1+int(max(float(r['ci95_high'] or r['mean_improvement_pct']) for r in data)//5))
        x = lambda t:left + (right-left)*t/maximum_ms
        y = lambda v:bottom-(bottom-top)*(v-ymin)/(ymax-ymin)
        draw.text((left,198),f'{n} customers',fill='#172D3E',font=bold(39))
        for pct in range(ymin,ymax+1,5):
            draw.line((left,y(pct),right,y(pct)),fill='#DFE7EB',width=2)
            draw.text((left-18,y(pct)),str(pct),anchor='rm',fill='#465D68',font=face(25))
        for time in sorted({int(r['checkpoint_ms']) for r in data}):
            draw.line((x(time),bottom,x(time),bottom+10),fill='#617581',width=2)
            draw.text((x(time),bottom+18),format(time/1000,'.3g'),anchor='mt',fill='#465D68',font=face(25))
        draw.line((left,top,left,bottom,right,bottom),fill='#546B78',width=3)
        draw.text(((left+right)//2,bottom+65),'Wall-clock checkpoint (s)',anchor='mt',fill='#465D68',font=face(26))
        for method,color in colors.items():
            series = [r for r in sorted(data,key=lambda r:int(r['checkpoint_ms'])) if r['method']==method]
            points = [(x(int(r['checkpoint_ms'])),y(float(r['mean_improvement_pct']))) for r in series]
            if not points:
                continue
            draw.line(points,fill=color,width=6 if method=='hybrid' else 4)
            for (px,py), row in zip(points,series):
                if row['ci95_low']:
                    lo, hi = y(float(row['ci95_low'])), y(float(row['ci95_high']))
                    draw.line((px,lo,px,hi),fill=color,width=2)
                    draw.line((px-5,lo,px+5,lo),fill=color,width=2)
                    draw.line((px-5,hi,px+5,hi),fill=color,width=2)
                draw.ellipse((px-6,py-6,px+6,py+6),fill=color,outline='white',width=1)

    for idx,(method,color) in enumerate(colors.items()):
        pos = 160+idx*392
        draw.line((pos,927,pos+55,927),fill=color,width=6)
        draw.text((pos+68,927),labels[method],anchor='lm',fill='#253E4D',font=face(26))
    image.save(directory/'figure3_convergence.png',dpi=(300,300))


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('stage',choices=('screen','formal'))
    args=parser.parse_args()
    plot(root / ('algorithm_screening' if args.stage=='screen' else 'algorithm_formal'),args.stage=='screen')
