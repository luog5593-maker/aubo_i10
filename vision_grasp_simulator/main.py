import sys
import traceback
from PySide6.QtWidgets import QApplication, QMessageBox
from app.main_window import MainWindow

if __name__ == '__main__':
    app = QApplication(sys.argv)
    try:
        win = MainWindow(); win.show(); sys.exit(app.exec())
    except Exception as exc:
        traceback.print_exc()
        QMessageBox.critical(None, '启动错误', f'软件启动失败：{exc}')
        raise
