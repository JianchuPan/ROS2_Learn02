import threading
import requests

class Download:
    def downlod(self, url,callback):
        print(f"线程：『{threading.get_ident()}』开始下载：{url}")
        reponse = requests.get(url)
        reponse.encoding = 'utf-8'
        callback(url,reponse.text)

    def start_download(self,url,callback):
        thread = threading.Thread(target=self.downlod,args=(url,callback))
        thread.start()

def download_finish_callback(url,result):
    print(f"『{url}』下载完成：,共{len(result)}字，内容为：{result[:10]}...")

def main():
    d = Download()
    d.start_download("http://www.baidu.com",download_finish_callback)
    d.start_download("http://www.sina.com.cn",download_finish_callback)
    d.start_download("http://www.163.com",download_finish_callback)
