from kivy.app import App
from kivy.uix.widget import Widget


from ntprop import NTProperty

NTProperty.default_team(2165)

class Dashboard(App):
    def build(self):
        return Widget()


if __name__ == "__main__":
    Dashboard().run()