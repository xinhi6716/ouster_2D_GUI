import cv2
import numpy as np
from ouster.sdk import client, pcap
from contextlib import closing
import datetime
import os

class SOIC_Ouster:
    def __init__(self,pcap_path:str,metadata_path:str) -> None:
        # 檢查檔案路徑
        if not os.path.isfile(pcap_path):
            raise FileNotFoundError('pcap_path not found')
        if not os.path.isfile(metadata_path):
            raise FileNotFoundError('metadata_path not found')

        self.pcap_path = pcap_path
        self.metadata_path = metadata_path
        
        with open(metadata_path, 'r') as f:
            self.info = client.SensorInfo(f.read())

        self.source = pcap.Pcap(pcap_path, self.info)
        self.scans_data = client.Scans(self.source)
        self.scans_list = []

    def scans_to_list(self) -> None:
        print("Converting scans iter to list")
        from tqdm import tqdm
        self.scans_list = [scan for scan in tqdm(self.scans_data)]
    
    def _process_frame_data_to_2d(self,frame_data) -> np.ndarray:
        # 提取範圍數據和反射率數據
        range_data = frame_data.field(client.ChanField.RANGE)
        reflectivity_data = frame_data.field(client.ChanField.REFLECTIVITY)

        # 校正數據
        range_data = client.destagger(self.info, range_data)
        reflectivity_data = client.destagger(self.info, reflectivity_data)
        
        # 過濾數據以減少噪點
        range_data = np.where((range_data > 0) & (range_data < np.percentile(range_data, 99)), range_data, 0)
        
        # 用反射率數據生成圖像
        image = reflectivity_data.astype(np.uint8)
        # print(image.shape) # (128,4096)

        # 調整圖像大小以符合視頻輸出的尺寸，手動拉伸分辨率以適應肉眼觀看
        resized_image = cv2.resize(image, (1440, 240), interpolation=cv2.INTER_LINEAR)
        return resized_image 
    
    def show(self,time_stamp:bool=False) -> None:
        if len(self.scans_list) == 0:
            self.scans_to_list()

        start_time = datetime.datetime.now()

        for fram_data in self.scans_list:
            frame = self._process_frame_data_to_2d(fram_data)
            
            if time_stamp:
                # 計算經過的時間
                elapsed_time = datetime.datetime.now() - start_time
                time_stamp = str(elapsed_time)[:-7]  # 刪除微秒部分

                # 在圖像上添加時間戳
                cv2.putText(frame, time_stamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # 顯示即時影像
            cv2.imshow('Frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
        cv2.destroyAllWindows()
    
    def save(self,output_path:str) -> None:
        print(f"Saving to {output_path}")
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(output_path, fourcc, self.info.format.fps, (1440, 240))
        
        for fram_data in self.scans_list:
            frame = self._process_frame_data_to_2d(fram_data)
            # 單通道轉三通道，以便寫入影片
            colored_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            out.write(colored_frame)
    
        out.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':

    soic_ouster = SOIC_Ouster('../A1.pcap','../A1.json')
    soic_ouster.show(time_stamp=True)
    soic_ouster.save('output.avi')