import requests, json, csv

post_data = {
    "gubun": "E",
    "link_idx": "B",
    "link_idx2": "",
    "page": 1
}

session = requests.Session()

commit_buffer = []

for char in range(ord("A"), ord("Z")+1):
    print("Processing for: " + chr(char))
    post_data["link_idx"] = chr(char)
    response = session.post("http://www.koicd.kr/2016/easyFind/v7.json", post_data)
    response_data = json.loads(response.text)["list"]

    for item in response_data:
        code = item["code_title"]
        kor_text = item["kor_text"]
        eng_text = item["eng_text"]
        item_tuple = (code, kor_text, eng_text)
        commit_buffer.append(item_tuple)

with open("./codes.csv", "w") as fp:
    csv_writer = csv.writer(fp, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for row in commit_buffer:
        csv_writer.writerow(row)