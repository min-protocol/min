
import model
import json
import argparse
import logging
import jinja2

if __name__ == "__main__":
    parser = argparse.ArgumentParser("Parse MIN JSON file and build signal handler")
    parser.add_argument('-i', dest='jsonfile', default='sample.json')
    parser.add_argument('--log', dest='loglevel', default=None)

    results = parser.parse_args()
    if results.loglevel:
        logging.basicConfig(filename='mintool.log',
                            format='%(asctime)s.%(msecs)03d %(message)s',
                            datefmt='%Y-%m-%d %H:%M:%S',
                            level=getattr(logging, results.loglevel.upper()))

    f = open(results.jsonfile)
    if f:
        data = json.load(f)

        signals = data["signals"]
        frames = data["frames"]
        input = data["input"]
        output = data["output"]
        network = data["network"]

        if signals:
            for signal_json in signals:
                s = model.Signal(signal_json)

        if frames:
            for frame_json in frames:
                f = model.Frame(frame_json)
                f.pack_signals()

        if input:
            for handle in input:
                f = model.Frame.get(handle)
                f.input = True

        if output:
            for handle in output:
                f = model.Frame.get(handle)
                f.output = True

        try:
            min_period = float(network["min_period"])
            model.Frame.min_period = min_period
        except:
            raise ValueError("MIN period in milliseconds required")

        model.Frame.allocate_min_ids()

        template_loader = jinja2.FileSystemLoader(searchpath=".")
        template_env = jinja2.Environment(loader=template_loader, lstrip_blocks=True, trim_blocks=True)

        template_vars = {
            "signals": model.Signal.signals,
            "frames": model.Frame.frames
        }

        codegen_h = template_env.get_template('codegen.h')
        codegen_c = template_env.get_template('codegen.c')

        gen_h = open("gen.h", mode="w")
        gen_c = open("gen.c", mode="w")

        gen_h.write(codegen_h.render(template_vars))
        gen_c.write(codegen_c.render(template_vars))
