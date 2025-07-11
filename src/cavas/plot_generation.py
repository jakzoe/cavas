from .spectrum_plot import SpectrumPlot
from .settings import PlotSettings
from .settings import MeasurementSettings
import os, sys, io
import time
import shutil
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
import copy
from pathlib import Path


delete_old_pictures = True
# um schnell bestimmtes zu exkludieren
plot_general = True
plot_fluo = True
plot_interpolate_only = False
dont_plot_interpolate = False
assert (plot_interpolate_only and dont_plot_interpolate) is False
plot_time_slices = True

# nur bestimmtes plotten. Leer ist disable (alles plotten). Enthält Keyword, welches in dem Namen sein muss.
plot_list = []  # ["Gradiant", "Tageslicht", "Neutral"]
blacklist = False  # black- oder whitelist


def sync_messungen_pics():

    # #!/bin/bash
    # rm -r messungen_pics/
    # # damit die Bilder die gleiche Struktur behalten
    # rsync -av --exclude='*.npz' --exclude='*.json' messungen/ messungen_pics/
    messungen_pics_path = Path("messungen_pics")
    if messungen_pics_path.exists():
        shutil.rmtree(messungen_pics_path)

    source = Path("messungen")
    destination = Path("messungen_pics")

    for root, dirs, files in os.walk(source):
        rel_path = Path(root).relative_to(source)
        dest_dir = destination / rel_path
        dest_dir.mkdir(parents=True, exist_ok=True)

        for file in files:
            if file.endswith(".npz") or file.endswith(".json"):
                continue

            src_file = Path(root) / file
            dest_file = dest_dir / file
            shutil.copy2(src_file, dest_file)


def make_plots(path, name):

    # damit nicht alles durcheinander ist (wegen multiprocessing)
    original_stdout = sys.stdout
    sys.stdout = io.StringIO()

    # grüner Text (\033[ ist Escape sequence start, 32m Green color code, 4m underline, 0m color reset)
    print(f"{path}:")
    print("\033[32m\033[4m" + name + "\033[0m")

    p_settings = []

    m_settings = MeasurementSettings.from_json(os.path.join(path, name + ".json"))

    # Generellen Durchschnitt plotten
    if plot_general:
        p_settings.append([PlotSettings(path, name, smooth=True)])

    # Fluoreszenz-Peak plotten (ca. zwischen 720 und 740 nm bei Chlorophyll)
    if plot_fluo:
        first_len = len(p_settings)
        p_settings.extend(
            (
                [
                    PlotSettings(
                        path,
                        name,
                        smooth=True,
                        single_wav=730,
                        scatter=True,
                    )
                ],
                [
                    PlotSettings(
                        path,
                        name,
                        smooth=True,
                        single_wav=740,
                        scatter=True,
                    )
                ],
                [
                    PlotSettings(
                        path,
                        name,
                        smooth=True,
                        single_wav=750,
                        scatter=True,
                    )
                ],
                [
                    # PlottingSettings(
                    #     path,
                    #     name,
                    #     smooth=True,
                    #     single_wav=750,
                    #     scatter=True,
                    # ),
                    PlotSettings(
                        path,
                        name,
                        smooth=True,
                        single_wav=530,
                        scatter=True,
                    ),
                ],
            )
        )
        last_len = len(p_settings)
        if not dont_plot_interpolate:
            for i in range(first_len, last_len):
                inte_setting = copy.deepcopy(p_settings[i][0])
                inte_setting.interpolate = True
                p_settings.append([inte_setting])

        if plot_interpolate_only:
            remove_setings = []
            for i in range(first_len, last_len):
                remove_setings.append(p_settings[i])
            for setting in remove_setings:
                p_settings.remove(setting)

    # # einzelne Zeitabschnitte plotten
    if plot_time_slices:
        p_settings.append(
            [
                PlotSettings(
                    path,
                    name,
                    smooth=True,
                    interval_start=0,
                    interval_end=1 / 3,
                    scatter=False,
                    line_style="-",
                    color="black",
                ),
                PlotSettings(
                    path,
                    name,
                    smooth=True,
                    interval_start=1 / 3,
                    interval_end=2 / 3,
                    line_style="--",
                    color="red",
                ),
                PlotSettings(
                    path,
                    name,
                    smooth=True,
                    interval_start=2 / 3,
                    interval_end=3 / 3,
                    line_style=":",
                    color="blue",
                ),
            ]
        )

    for p in p_settings:
        SpectrumPlot.plot_results(p, m_settings, show_plots=False)

    sys.stdout.flush()
    # sys.stderr.flush()
    output = sys.stdout.getvalue()
    sys.stdout = original_stdout
    print(output)
    # print(sys.stderr.getvalue())


if __name__ == "__main__":

    # path = r"messungen/Gradiant_Test/Kontinuierlich/"
    # name = r"test-messung"
    # m_settings = MeasurementSettings.from_json(os.path.join(path, name + ".json"))
    # # m_settings.print_status()

    # p = [
    #     PlottingSettings(
    #         path,
    #         name,
    #         smooth=True,
    #         # single_wav=750,
    #         grad_start=2,
    #         grad_end=4,
    #         scatter=True,
    #     )
    # ]
    # Laserplot().plot_results(p, m_settings, show_plots=False)

    # exit()

    # die ganzen Symblinks löschen
    if delete_old_pictures:
        try:
            shutil.rmtree("plots/")
        except FileNotFoundError:
            print("plots dir was already deleted")

    paths = []
    root = "messungen/"
    for dir in os.listdir(root):
        for subdir in os.listdir(os.path.join(root, dir)):
            paths.append(os.path.join(root, dir, subdir))

    tasks = []

    for path in paths:

        names = [
            os.path.splitext(f)[0]
            for f in os.listdir(path)
            if (f.endswith((".npz")) and "overwrite-messung" not in f)
        ]

        if delete_old_pictures:
            pic_names = [
                os.path.splitext(f)[0] for f in os.listdir(path) if f.endswith((".png"))
            ]
            for pic_name in pic_names:
                os.remove(os.path.join(path, pic_name + ".png"))

        # ob das Element in der white/blacklist ist
        if plot_list and (
            (blacklist and any(ele in path for ele in plot_list))
            or (not blacklist and any(ele not in path for ele in plot_list))
        ):
            # print(f"skipping {path}")
            continue

        for name in names:
            tasks.append((path, name))

    # for task in tasks:
    #     make_plots(*task)

    # exit()

    # lambda geht nicht...
    def worker(args):
        return make_plots(*args)

    start_time = time.time()

    try:
        with ProcessPoolExecutor(max_workers=int(os.cpu_count() / 1.5)) as executor:
            executor.map(worker, tasks)
    except KeyboardInterrupt:
        multiprocessing.active_children()
        for p in multiprocessing.active_children():
            p.terminate()
        exit()

    print(f"took: {time.time() - start_time:.2f} s")

    sync_messungen_pics()
    # 12: took: 44.83 s
    # 8: took: 49.54 s
    # 4: took: 83.07 s
    # 1: took: 200.00 s
