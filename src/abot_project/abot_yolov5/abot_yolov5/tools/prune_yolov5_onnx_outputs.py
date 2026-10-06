#!/usr/bin/env python3
import argparse
import os

import onnx


def main():
    parser = argparse.ArgumentParser(
        description="Keep only the decoded YOLOv5 prediction output in ONNX."
    )
    parser.add_argument("model")
    parser.add_argument("--output-name", default="output")
    arguments = parser.parse_args()

    model_path = os.path.abspath(arguments.model)
    model = onnx.load(model_path)
    selected = [
        value
        for value in model.graph.output
        if value.name == arguments.output_name
    ]
    if len(selected) != 1:
        names = [value.name for value in model.graph.output]
        raise RuntimeError(
            "expected exactly one ONNX output named %s; found %s"
            % (arguments.output_name, names)
        )
    output = onnx.ValueInfoProto()
    output.CopyFrom(selected[0])
    del model.graph.output[:]
    model.graph.output.append(output)
    onnx.checker.check_model(model)

    temporary_path = model_path + ".single-output.tmp"
    onnx.save(model, temporary_path)
    os.replace(temporary_path, model_path)
    print("kept ONNX output: %s" % arguments.output_name)


if __name__ == "__main__":
    main()
