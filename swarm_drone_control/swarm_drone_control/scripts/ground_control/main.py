
import sys
import os
import tempfile
import folium  # pip install folium

# Must be set BEFORE QApplication is created
os.environ['QTWEBENGINE_CHROMIUM_FLAGS'] = '--disable-web-security --allow-file-access-from-files'

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEngineSettings, QWebEngineProfile
from PyQt5.QtCore import QUrl

"""
Folium in PyQt5 - Ground Control Station Map
"""
class MyApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Ground Control Map')
        self.window_width, self.window_height = 1600, 1200
        self.setMinimumSize(self.window_width, self.window_height)

        layout = QVBoxLayout()
        self.setLayout(layout)

        coordinate = (37.8199286, -122.4782551)
        m = folium.Map(
            tiles='OpenStreetMap',
            zoom_start=13,
            location=coordinate
        )

        # Save map to a temp file and load via file:// URL
        self._tmp = tempfile.NamedTemporaryFile(
            suffix='.html', delete=False, mode='w', encoding='utf-8'
        )
        m.save(self._tmp.name)
        self._tmp.close()

        # Apply permissive settings on the default profile
        profile = QWebEngineProfile.defaultProfile()
        profile.settings().setAttribute(QWebEngineSettings.LocalContentCanAccessRemoteUrls, True)
        profile.settings().setAttribute(QWebEngineSettings.LocalContentCanAccessFileUrls, True)
        profile.settings().setAttribute(QWebEngineSettings.AllowRunningInsecureContent, True)

        webView = QWebEngineView()
        webView.setUrl(QUrl.fromLocalFile(self._tmp.name))
        layout.addWidget(webView)

    def closeEvent(self, event):
        try:
            os.unlink(self._tmp.name)
        except Exception:
            pass
        super().closeEvent(event)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyleSheet('''
        QWidget {
            font-size: 35px;
        }
    ''')

    myApp = MyApp()
    myApp.show()

    try:
        sys.exit(app.exec_())
    except SystemExit:
        print('Closing Window...')