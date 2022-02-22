import requests


def main():
    response = requests.get('http://192.168.104.177:5001/config')
    if response.ok:
        print(response.json())

if __name__ == '__main__':
    main()
