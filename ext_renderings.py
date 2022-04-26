import pyqtgraph.opengl as gl


# Meant to clean up code and have table rendered here

# Create opengl Window
def createWindow():
    # ---- Set Up Window ----
    w = gl.GLViewWidget()
    w.opts['distance'] = 150
    w.setWindowTitle('Ping-Pong Robot-Control')
    w.setGeometry(0, 110, 1280, 720)
    w.show()
    return w


# Render table in window
def renderTable(w):
    # Numbers that work well (in inches) with the graph and keep proper proportions
    xtable = 108
    ytable = 60

    xnet = 66
    ynet = 6  # Kinda like a z coord after 90 degree rotation

    # ---- Plot Table and Net ----
    table = gl.GLGridItem()  # Create table
    table.translate(xtable / 2, ytable / 2, 0)  # Move to correct coord
    table.setSize(xtable, ytable)  # Size table
    table.setSpacing(6, 6)  # Size grid spaces
    w.addItem(table)  # Add table to view

    net = gl.GLGridItem()  # Create net
    net.rotate(90, 1, 0, 0)  # Rotate plain
    net.rotate(90, 0, 0, 1)  # Rotate plain
    net.translate(xtable / 2, ytable / 2, ynet / 2)  # Move to correct pos
    net.setSize(xnet, ynet)  # Size table
    w.addItem(net)  # Add table to view
