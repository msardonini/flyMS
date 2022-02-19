import requests

def main():
    response = requests.get('http://localhost:5000/config')
    if response.ok:
        print(response.json())

if __name__ == '__main__':
    main()