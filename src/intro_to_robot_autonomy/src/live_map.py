from dash import Dash, dcc, html
from dash.dependencies import Input, Output
import plotly.express as px
import pandas as pd
import rospy
from sensor_msgs.msg import NavSatFix
import threading

# Global değişkenler
latest_lat = None
latest_lon = None
new_message_received = False  # Yeni mesaj gelip gelmediğini kontrol etmek için

# ROS callback: Konumları günceller
def gps_callback(msg):
    global latest_lat, latest_lon, new_message_received
    latest_lat = msg.latitude
    latest_lon = msg.longitude
    new_message_received = True

# ROS Subscriber için thread içinde çalışacak fonksiyon
def ros_subscriber_thread():
    rospy.Subscriber('/gps/fix', NavSatFix, gps_callback)
    rospy.spin()

# Dash uygulamasını başlat
app = Dash(__name__)

app.layout = html.Div([
    dcc.Graph(id='map'),
    dcc.Interval(
        id='interval-component',
        interval=500,  # 500ms yenileme hızı
        n_intervals=0
    )
])

# Dash callback: Haritayı yalnızca yeni bir mesaj alındığında günceller
@app.callback(
    Output('map', 'figure'),
    [Input('interval-component', 'n_intervals')]
)
def update_map(n_intervals):
    global latest_lat, latest_lon, new_message_received

    # Haritayı yalnızca yeni bir ROS mesajı varsa güncelle
    if not new_message_received or latest_lat is None or latest_lon is None:
        raise dash.exceptions.PreventUpdate

    # Gelen veriyi DataFrame olarak hazırla
    data = {"Lat": [latest_lat], "Long": [latest_lon], "Address": ["ROS GPS"]}
    df = pd.DataFrame(data)

    # Haritayı oluştur
    fig = px.scatter_mapbox(df,
                            lat="Lat",
                            lon="Long",
                            hover_name="Address",
                            zoom=18,
                            height=600,
                            width=800,
                            size=[10])

    fig.update_layout(mapbox_style="open-street-map")
    fig.update_layout(margin={"r": 0, "t": 0, "l": 0, "b": 0})

    # Mesajı işledikten sonra durumu sıfırla
    new_message_received = False

    return fig

if __name__ == '__main__':
    # Ana thread'de ROS node'u başlat
    rospy.init_node('dash_gps_subscriber', anonymous=True)

    # ROS Subscriber işlemini ayrı bir thread'de çalıştır
    ros_thread = threading.Thread(target=ros_subscriber_thread)
    ros_thread.daemon = True
    ros_thread.start()

    # Dash uygulamasını çalıştır
    app.run_server(debug=False)

