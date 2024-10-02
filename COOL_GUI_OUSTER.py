import customtkinter as ctk
import ttkbootstrap as ttk
from tkinter import filedialog
from PIL import Image, ImageTk
import cv2
import numpy as np
from ouster.sdk import client, pcap
from contextlib import closing
import threading
import os
from datetime import datetime
import open3d as o3d
from more_itertools import time_limited

class LiDARPlayer(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("LiDAR Player")
        self.geometry("1920x1080")

        # 初始化變量
        self.pcap_path = None
        self.metadata_path = None
        self.hostname = None
        self.lidar_port = None
        self.imu_port = None
        self.playing = False
        self.realtime = False
        self.collecting_data = False
        self.thread = None
        self.smart_mode = ctk.BooleanVar()
        self.save_dir = None  # 新增一個變數來儲存保存目錄

        self.create_widgets()

    def create_widgets(self):
        """
        創建界面控件，包括視訊框架、Canvas、按鈕等
        """
        # 視訊框架
        self.video_frame = ctk.CTkFrame(self, width=1440, height=240)
        self.video_frame.pack(pady=20)

        # Canvas 用於顯示2D影像
        self.canvas_2d = ctk.CTkCanvas(self.video_frame, width=1440, height=240, bg="black")
        self.canvas_2d.pack()

        # 點雲框架
        self.pointcloud_frame = ctk.CTkFrame(self, width=1440, height=480)
        self.pointcloud_frame.pack(pady=20)

        # 檔案資訊標籤
        self.file_info = ctk.CTkLabel(self, text="", font=("Arial", 12))
        self.file_info.pack(pady=10)

        # 按鈕框架
        self.button_frame = ctk.CTkFrame(self)
        self.button_frame.pack(pady=10, fill="x", side="bottom")

        # 選擇PCAP文件按鈕
        self.pcap_button = ctk.CTkButton(self.button_frame, text="Select PCAP File", command=self.select_pcap_file, width=200, height=50)
        self.pcap_button.grid(row=0, column=0, padx=10, pady=10)

        # 選擇JSON文件按鈕
        self.json_button = ctk.CTkButton(self.button_frame, text="Select JSON File", command=self.select_json_file, width=200, height=50)
        self.json_button.grid(row=0, column=1, padx=10, pady=10)

        # 實時模式選擇按鈕
        self.realtime_button = ctk.CTkButton(self.button_frame, text="Enable Real-time Mode", command=self.enable_realtime_mode, width=200, height=50)
        self.realtime_button.grid(row=0, column=2, padx=10, pady=10)

        # 智慧模式選擇按鈕
        self.smart_mode_button = ctk.CTkCheckBox(self.button_frame, text="Smart Mode", variable=self.smart_mode)
        self.smart_mode_button.grid(row=1, column=0, padx=10, pady=10)

        # 主機名輸入框
        self.hostname_entry = ctk.CTkEntry(self.button_frame, placeholder_text="Hostname or IP Address", width=200, height=50)
        self.hostname_entry.grid(row=1, column=1, padx=10, pady=10)

        # LiDAR端口輸入框
        self.lidar_port_entry = ctk.CTkEntry(self.button_frame, placeholder_text="LiDAR Port", width=200, height=50)
        self.lidar_port_entry.grid(row=1, column=2, padx=10, pady=10)

        # IMU端口輸入框
        self.imu_port_entry = ctk.CTkEntry(self.button_frame, placeholder_text="IMU Port", width=200, height=50)
        self.imu_port_entry.grid(row=1, column=3, padx=10, pady=10)

        # 數據蒐集模式選擇按鈕
        self.collect_data_button = ctk.CTkCheckBox(self.button_frame, text="Collect Data", command=self.toggle_collect_data)
        self.collect_data_button.grid(row=1, column=4, padx=10, pady=10)

        # 播放按鈕
        self.play_button = ctk.CTkButton(self.button_frame, text="Play", command=self.play, state=ctk.DISABLED, width=200, height=50)
        self.play_button.grid(row=2, column=0, padx=10, pady=10)

        # 暫停按鈕
        self.pause_button = ctk.CTkButton(self.button_frame, text="Pause", command=self.pause, state=ctk.DISABLED, width=200, height=50)
        self.pause_button.grid(row=2, column=1, padx=10, pady=10)

    def select_pcap_file(self):
        """
        選擇PCAP文件並更新相關信息
        """
        self.pcap_path = filedialog.askopenfilename(title="Select PCAP File", filetypes=[("PCAP Files", "*.pcap")])
        if self.pcap_path and self.smart_mode.get():
            self.auto_select_json_file()
        self.update_file_info()
        self.check_ready()

    def select_json_file(self):
        """
        選擇JSON文件並更新相關信息
        """
        self.metadata_path = filedialog.askopenfilename(title="Select JSON File", filetypes=[("JSON Files", "*.json")])
        if self.metadata_path and self.smart_mode.get():
            self.auto_select_pcap_file()
        self.update_file_info()
        self.check_ready()

    def auto_select_json_file(self):
        """
        自動選擇與PCAP文件同名的JSON文件
        """
        json_path = os.path.splitext(self.pcap_path)[0] + ".json"
        if os.path.exists(json_path):
            self.metadata_path = json_path

    def auto_select_pcap_file(self):
        """
        自動選擇與JSON文件同名的PCAP文件
        """
        pcap_path = os.path.splitext(self.metadata_path)[0] + ".pcap"
        if os.path.exists(pcap_path):
            self.pcap_path = pcap_path

    def update_file_info(self):
        """
        更新檔案資訊標籤
        """
        file_info_text = f"PCAP: {self.pcap_path}\nJSON: {self.metadata_path}"
        self.file_info.configure(text=file_info_text)

    def enable_realtime_mode(self):
        """
        啟用實時模式
        """
        self.realtime = True
        self.update_file_info()
        self.check_ready()

    def toggle_collect_data(self):
        """
        切換數據蒐集模式
        """
        self.collecting_data = not self.collecting_data
        if self.collecting_data:
            self.save_dir = filedialog.askdirectory(title="Select Directory to Save Data")

    def check_ready(self):
        """
        檢查是否準備好播放
        """
        if self.pcap_path or self.metadata_path or self.realtime:
            self.play_button.configure(state=ctk.NORMAL)

    def play(self):
        """
        開始播放PCAP或實時數據
        """
        self.playing = True
        self.play_button.configure(state=ctk.DISABLED)
        self.pause_button.configure(state=ctk.NORMAL)
        if self.realtime:
            self.hostname = self.hostname_entry.get()
            self.lidar_port = int(self.lidar_port_entry.get())
            self.imu_port = int(self.imu_port_entry.get())
            self.thread = threading.Thread(target=self.visualize_realtime)
        else:
            self.thread = threading.Thread(target=self.visualize_pcap_live)
        self.thread.start()

    def pause(self):
        """
        暫停播放
        """
        self.playing = False
        self.play_button.configure(state=ctk.NORMAL)
        self.pause_button.configure(state=ctk.DISABLED)

    def process_scan_to_2d(self, scan, metadata):
        """
        將掃描數據處理為2D圖像並顯示距離資訊
        """
        # 從掃描數據中提取範圍數據和反射率數據
        range_data = scan.field(client.ChanField.RANGE)
        reflectivity_data = scan.field(client.ChanField.REFLECTIVITY)
        # 對數據進行去擾碼處理
        range_data = client.destagger(metadata, range_data)
        reflectivity_data = client.destagger(metadata, reflectivity_data)
        # 過濾範圍數據以減少噪點
        range_data = np.where((range_data > 0) & (range_data < np.percentile(range_data, 99)), range_data, 0)
        # 將反射率數據轉換為8位灰度圖像
        image = reflectivity_data.astype(np.uint8)
        
        # 自適應直方圖均衡化，增強圖像對比度
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        image = clahe.apply(image)
        
        # 將灰度圖像轉換為彩色圖像
        colored_image = cv2.applyColorMap(image, cv2.COLORMAP_JET)
        
        # 調整圖像大小至1960x240
        resized_image = cv2.resize(colored_image, (1440, 240), interpolation=cv2.INTER_LINEAR)
        
        # 將範圍數據調整為顯示範圍
        distance_data = range_data / 1000.0  # 假設範圍數據的單位是毫米
        distance_data_resized = cv2.resize(distance_data, (1440, 240), interpolation=cv2.INTER_LINEAR)
        
        # 將距離信息疊加到圖像上
        for i in range(0, 1440, 50):  # 每隔50像素顯示一次
            for j in range(0, 240, 50):
                distance = distance_data_resized[j, i]
                if distance > 0:
                    cv2.putText(resized_image, f"{distance:.1f}m", (i, j), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
        return resized_image  # 返回處理後的圖像

    def update_canvas(self, frame):
        """
        更新Canvas上的圖像
        """
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(frame)
        imgtk = ImageTk.PhotoImage(image=img)
        self.canvas_2d.create_image(0, 0, anchor="nw", image=imgtk)
        self.canvas_2d.image = imgtk

    def visualize_pcap_live(self):
        """
        可視化PCAP文件中的數據
        """
        with open(self.metadata_path, 'r') as f:
            metadata = client.SensorInfo(f.read())

        source = pcap.Pcap(self.pcap_path, metadata)
        scans = client.Scans(source)

        frame_count = 0
        with closing(scans) as scan_source:
            for scan in scan_source:
                if not self.playing:
                    break
                frame = self.process_scan_to_2d(scan, metadata)
                self.update_canvas(frame)
                frame_count += 1

        cv2.destroyAllWindows()

    def visualize_realtime(self):
        """
        可視化實時數據並進行數據收集
        """
        with closing(client.Sensor(self.hostname, self.lidar_port, self.imu_port, buf_size=640)) as source:
            metadata = source.metadata
            scans = client.Scans(source)

            if self.collecting_data:
                time_part = datetime.now().strftime("%Y%m%d_%H%M%S")
                fname_base = f"{metadata.prod_line}_{metadata.sn}_{metadata.mode}_{time_part}"
                if self.save_dir:
                    fname_base = os.path.join(self.save_dir, fname_base)
                
                print(f"Saving sensor metadata to: {fname_base}.json")
                source.write_metadata(f"{fname_base}.json")

                print(f"Writing to: {fname_base}.pcap (Ctrl-C to stop early)")
                source_it = source #不限時錄製 直到按下停止
                ### source_it = time_limited(60, source)  #開啟限時錄製 60 seconds for example
                n_packets = pcap.record(source_it, f"{fname_base}.pcap")
                print(f"Captured {n_packets} packets")

            frame_count = 0
            for scan in scans:
                if not self.playing:
                    break
                frame = self.process_scan_to_2d(scan, metadata)
                self.update_canvas(frame)
                frame_count += 1

        cv2.destroyAllWindows()

    def visualize_point_cloud(self, point_cloud):
        """
        可視化點雲數據
        """
        points = point_cloud.reshape(-1, 3).astype(np.float64)  # 確保數據類型是 float64 並重新塑形
        print(f"Visualizing point cloud with shape: {points.shape}, dtype: {points.dtype}")  # 調試信息
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Ouster Point Cloud")
        vis.get_render_option().point_size = 1
        vis.get_render_option().background_color = np.asarray([0, 0, 0])
        pcd.paint_uniform_color([1, 1, 1])  # 設置點的顏色為白色
        vis.add_geometry(pcd)
        vis.run()
        vis.destroy_window()

    def visualize_pcap_live_3d(self):
        """
        可視化PCAP文件中的3D點雲數據
        """
        with open(self.metadata_path, 'r') as f:
            metadata = client.SensorInfo(f.read())

        source = pcap.Pcap(self.pcap_path, metadata)
        scans = client.Scans(source)

        xyzlut = client.XYZLut(metadata)
        for scan in scans:
            if not self.playing:
                break
            point_cloud = xyzlut(scan)
            self.visualize_point_cloud(point_cloud)

if __name__ == "__main__":
    app = LiDARPlayer()
    app.mainloop()
