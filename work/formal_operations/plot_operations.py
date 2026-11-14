import csv
import argparse
import pathlib
import statistics

from PIL import Image, ImageDraw, ImageFont

from run_operations import OUTPUT


W, H = 1500, 920
FONT_DIR = pathlib.Path("C:/Windows/Fonts")
INK = "#172b39"
GREY = "#526570"
GRID = "#e1e8ea"
TEAL = "#00796e"
ORANGE = "#b45324"
VIOLET = "#655292"


def font(size, bold=False):
    return ImageFont.truetype(str(FONT_DIR / ("arialbd.ttf" if bold else "arial.ttf")), size)


def read(name):
    with (OUTPUT / name).open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


class Chart:
    def __init__(self, title, subtitle):
        self.canvas = Image.new("RGB", (W, H), "#ffffff")
        self.draw = ImageDraw.Draw(self.canvas)
        self.text(76, 56, title, 49, bold=True)
        self.text(76, 122, subtitle, 28, color=GREY)
        self.x0, self.x1, self.y0, self.y1 = 149, 1388, 215, 735

    def text(self, x, y, content, size=25, *, color=INK, bold=False, anchor="lt"):
        self.draw.text((x, y), content, font=font(size, bold), fill=color, anchor=anchor)

    def axes(self, x_ticks, y_ticks, x_label, y_label):
        self.xmin, self.xmax = x_ticks[0], x_ticks[-1]
        self.ymin, self.ymax = y_ticks[0], y_ticks[-1]
        self.text(77, 176, y_label, 27, bold=True)
        for tick in y_ticks:
            _, y = self.point(self.xmin, tick)
            self.draw.line((self.x0, y, self.x1, y), fill=GRID, width=2)
            self.text(self.x0 - 15, y, f"{tick:g}", 24, anchor="rm")
        self.draw.line((self.x0, self.y0, self.x0, self.y1, self.x1, self.y1), fill=INK, width=3)
        for tick in x_ticks:
            x, _ = self.point(tick, self.ymin)
            self.draw.line((x, self.y1, x, self.y1 + 9), fill=INK, width=3)
            self.text(x, self.y1 + 17, f"{tick:g}", 25, anchor="mt")
        self.text((self.x0 + self.x1) / 2, self.y1 + 58, x_label, 27, bold=True, anchor="mt")

    def point(self, x, y):
        return (self.x0 + (x - self.xmin) * (self.x1 - self.x0) / (self.xmax - self.xmin),
                self.y1 - (y - self.ymin) * (self.y1 - self.y0) / (self.ymax - self.ymin))

    def dots(self, data, color, hollow=False):
        for x, y in data:
            a, b = self.point(x, y)
            self.draw.ellipse((a - 8, b - 8, a + 8, b + 8), fill="#ffffff" if hollow else color,
                              outline=color, width=4)

    def series(self, data, color, dashed=False):
        points = [self.point(x, y) for x, y in data]
        if dashed:
            for (a, b), (c, d) in zip(points, points[1:]):
                for start in range(0, 12, 2):
                    left, right = start / 12, (start + 1) / 12
                    self.draw.line((a + left * (c - a), b + left * (d - b),
                                    a + right * (c - a), b + right * (d - b)), fill=color, width=5)
        else:
            self.draw.line(points, fill=color, width=5)
        self.dots(data, color, dashed)

    def legend(self, x, y, label, color, hollow=False):
        self.draw.ellipse((x, y + 5, x + 15, y + 20), fill="#ffffff" if hollow else color,
                          outline=color, width=3)
        self.text(x + 26, y, label, 25)

    def note(self, caption):
        self.text(77, 869, caption, 23, color=GREY)

    def save(self, file):
        self.canvas.save(OUTPUT / file, dpi=(240, 240))


def fleet():
    summary = read("summary.csv")
    nest = read("fleet_feasible_envelope.csv")
    means = [statistics.mean(float(row["best_known_cost_cny"]) for row in nest if int(row["drones"]) == d)
             for d in range(7)]
    direct = []
    for d in range(7):
        name = "reference" if d == 3 else f"fleet_{d}"
        row = next(row for row in summary if row["name"] == name and row["geometry"] == "ALL")
        assert row["valid"] == "36"
        direct.append(float(row["mean_objective"]))
    chart = Chart("Operating cost by fleet size", "36 matched synthetic instances · 350 ms per search")
    chart.axes(list(range(7)), [900, 950, 1000, 1050, 1100, 1150], "Available drones", "Mean cost (CNY per route)")
    chart.series(list(enumerate(direct)), ORANGE, dashed=True)
    chart.series(list(enumerate(means)), TEAL)
    chart.legend(868, 250, "Best known feasible envelope", TEAL)
    chart.legend(868, 287, "Independent searches", ORANGE, hollow=True)
    chart.note("Envelope reuses feasible routes from smaller fleets. Truck tare is fixed.")
    chart.save("Figure_4_fleet.png")


def payload():
    estimates = read("paired_effects.csv")
    chart = Chart("Payload limit and delivery cost",
                  "Paired differences from the 4.5 kg reference · 36 matched synthetic instances")
    chart.axes([3.5, 4.0, 4.5], [-10, 0, 10, 20, 30, 40, 50], "Payload limit (kg)",
               "Mean cost difference (CNY per route)")
    zero = chart.point(3.5, 0)[1]
    chart.draw.line((chart.x0, zero, chart.x1, zero), fill=GREY, width=2)
    for q in (3.5, 4.0):
        row = next(row for row in estimates if row["name"] == f"payload_{q:.1f}"
                   and row["geometry"] == "ALL" and row["metric"] == "objective")
        x, y = chart.point(q, float(row["mean"]))
        lo = chart.point(q, float(row["p2_5"]))[1]
        hi = chart.point(q, float(row["p97_5"]))[1]
        chart.draw.line((x, lo, x, hi), fill=TEAL, width=5)
        for cap in (lo, hi):
            chart.draw.line((x - 11, cap, x + 11, cap), fill=TEAL, width=5)
        chart.dots([(q, float(row["mean"]))], TEAL)
        chart.text(x + 18, y - 18, f'{float(row["mean"]):+.2f}', 27, color=TEAL, bold=True)
    chart.dots([(4.5, 0)], TEAL)
    chart.text(chart.x1 - 14, zero - 31, "Reference", 26, color=TEAL, bold=True, anchor="rt")
    chart.note("95% stratified paired bootstrap intervals. Three drones and 0.510 kWh are held fixed.")
    chart.save("Figure_5_payload.png")


def prices():
    raw = read("price_candidate_pool_summary.csv")
    chart = Chart("Truck dwell by waiting charge",
                  "Same feasible route pool in every price cell · grid factor 0.55 kg CO2e/kWh")
    charges = [0, .005, .010, .020]
    chart.axes(charges, [3000, 3500, 4000, 4500, 5000, 5500], "Waiting charge (CNY per second)",
               "Mean truck dwell (seconds per route)")
    for carbon, color, dashed in [(0.0, ORANGE, True), (1.056, VIOLET, False)]:
        values = []
        for c in charges:
            name = f"prices_{carbon:.3f}_{c:.3f}_0.55"
            row = next(row for row in raw if row["name"] == name and row["geometry"] == "ALL")
            assert row["cases"] == "36"
            values.append((c, float(row["mean_wait_s"])))
        chart.series(values, color, dashed)
        chart.legend(868, 247 if dashed else 284, f"Carbon charge {carbon:g} CNY/kg CO2e", color, dashed)
    chart.note("The least-cost known plan is selected for each of the same 36 instances in each cell.")
    chart.save("Figure_6_price_dwell.png")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=pathlib.Path, default=OUTPUT)
    OUTPUT = parser.parse_args().output
    fleet()
    payload()
    prices()
