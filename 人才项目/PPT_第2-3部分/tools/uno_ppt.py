from __future__ import annotations

import shutil
import socket
import subprocess
import tempfile
import time
from pathlib import Path

import uno
from com.sun.star.awt import Point, Size
from com.sun.star.beans import PropertyValue


def prop(name, value):
    property_value = PropertyValue()
    property_value.Name = name
    property_value.Value = value
    return property_value


def file_url(path):
    return uno.systemPathToFileUrl(str(Path(path).resolve()))


def cm(value):
    return int(round(value * 1000))


class ImpressSession:
    def __init__(self):
        self.profile = None
        self.process = None
        self.context = None
        self.desktop = None

    def __enter__(self):
        self.profile = Path(tempfile.mkdtemp(prefix="robofuzz-lo-profile-"))
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
                listener.bind(("127.0.0.1", 0))
                port = listener.getsockname()[1]

            command = [
                "soffice",
                "--headless",
                "--nologo",
                "--nodefault",
                "--norestore",
                "--nofirststartwizard",
                f"-env:UserInstallation={file_url(self.profile)}",
                f"--accept=socket,host=127.0.0.1,port={port};urp;StarOffice.ComponentContext",
            ]
            self.process = subprocess.Popen(
                command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

            resolver = uno.getComponentContext().ServiceManager.createInstanceWithContext(
                "com.sun.star.bridge.UnoUrlResolver", uno.getComponentContext()
            )
            connection_url = (
                f"uno:socket,host=127.0.0.1,port={port};urp;StarOffice.ComponentContext"
            )
            last_error = None
            for _ in range(80):
                try:
                    self.context = resolver.resolve(connection_url)
                    break
                except Exception as error:
                    last_error = error
                    time.sleep(0.1)
            else:
                raise RuntimeError(
                    "Unable to connect to the isolated LibreOffice instance"
                ) from last_error

            self.desktop = self.context.ServiceManager.createInstanceWithContext(
                "com.sun.star.frame.Desktop", self.context
            )
            return self
        except Exception:
            self.__exit__(None, None, None)
            raise

    def open(self, path):
        document = self.desktop.loadComponentFromURL(
            file_url(path),
            "_blank",
            0,
            (prop("Hidden", True), prop("ReadOnly", False)),
        )
        if document is None:
            raise RuntimeError(f"LibreOffice could not open presentation: {path}")
        return document

    def save_and_close(self, document):
        document.store()
        document.close(True)

    def close(self, document):
        document.close(True)

    def __exit__(self, exc_type, exc_value, traceback):
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        if self.profile is not None:
            shutil.rmtree(self.profile, ignore_errors=True)
        self.profile = None
        self.process = None
        self.context = None
        self.desktop = None


def clear_page(page):
    for index in range(page.getCount() - 1, -1, -1):
        page.remove(page.getByIndex(index))


def add_text(
    document,
    page,
    x,
    y,
    w,
    h,
    text,
    font_size,
    color=0x000000,
    bold=False,
    align=0,
    valign=0,
    border=None,
    fill=None,
    dash=False,
):
    shape = document.createInstance("com.sun.star.drawing.TextShape")
    shape.Position = Point(cm(x), cm(y))
    shape.Size = Size(cm(w), cm(h))
    page.add(shape)
    shape.String = text
    shape.CharFontName = "Microsoft YaHei"
    shape.CharHeight = float(font_size)
    shape.CharWeight = 150.0 if bold else 100.0
    shape.CharColor = color
    shape.ParaAdjust = align
    shape.TextVerticalAdjust = valign
    shape.TextAutoGrowHeight = False
    if fill is None:
        shape.FillStyle = 0
    else:
        shape.FillStyle = 1
        shape.FillColor = fill
    if border is None:
        shape.LineStyle = 0
    else:
        shape.LineStyle = 2 if dash else 1
        shape.LineColor = border
        shape.LineWidth = 70
    return shape


def color_prefix(shape, prefix, color=0xC00000):
    if not shape.String.startswith(prefix):
        raise ValueError(f"Text does not start with prefix: {prefix!r}")
    cursor = shape.createTextCursor()
    cursor.gotoStart(False)
    cursor.goRight(len(prefix), True)
    cursor.CharColor = color
    cursor.CharWeight = 150.0


def add_svg(session, document, page, path, x, y, w, h):
    graphic_provider = session.context.ServiceManager.createInstanceWithContext(
        "com.sun.star.graphic.GraphicProvider", session.context
    )
    graphic = graphic_provider.queryGraphic((prop("URL", file_url(path)),))
    if graphic is None:
        raise RuntimeError(f"LibreOffice could not load SVG: {path}")
    shape = document.createInstance("com.sun.star.drawing.GraphicObjectShape")
    shape.Position = Point(cm(x), cm(y))
    shape.Size = Size(cm(w), cm(h))
    shape.Graphic = graphic
    page.add(shape)
    return shape


def make_table(document, page, x, y, w, h, columns, rows):
    shape = document.createInstance("com.sun.star.drawing.TableShape")
    shape.Position = Point(cm(x), cm(y))
    shape.Size = Size(cm(w), cm(h))
    page.add(shape)
    table = shape.Model
    table.initialize(columns, rows)
    return shape, table


def border_line(color=0, width=35):
    line = uno.createUnoStruct("com.sun.star.table.BorderLine2")
    line.Color = color
    line.InnerLineWidth = 0
    line.OuterLineWidth = width
    line.LineDistance = 0
    line.LineWidth = width
    return line


def set_cell(
    cell,
    text,
    font_size=11.5,
    color=0,
    bold=False,
    fill=0xFFFFFF,
    align=0,
    valign=0,
):
    cell.String = text
    cell.CharFontName = "Microsoft YaHei"
    cell.CharHeight = float(font_size)
    cell.CharWeight = 150.0 if bold else 100.0
    cell.CharColor = color
    cell.FillColor = fill
    cell.ParaAdjust = align
    cell.TextVerticalAdjust = valign
    line = border_line()
    cell.TopBorder = line
    cell.BottomBorder = line
    cell.LeftBorder = line
    cell.RightBorder = line
