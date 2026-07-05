from PySide6.QtWidgets import QWidget,QVBoxLayout,QHBoxLayout,QPushButton,QTextEdit
class LogPanel(QWidget):
    def __init__(self):
        super().__init__(); lay=QVBoxLayout(self); self.text=QTextEdit(); self.text.setReadOnly(True)
        btn=QPushButton('清空日志'); btn.clicked.connect(self.text.clear); lay.addWidget(self.text); lay.addWidget(btn)
    def append(self,line): self.text.append(line)
