독일에서 만든 세그먼테이션 기반의 데이터 셋 자갈 길이 많은 독일의 도로를 학습 시켰기에 한국 도로에서의 감지율이 낮은 편임

실제 구현 할때는 gtFine_trainvaltest 데이터 셋 (11giga byte)를 받아서 학습을 진행하였는데 이보다 더 큰 데이터 셋을 사용한다면 좋을 수 있음

Yolov8m-seg를 통한 학습을 진행하였으나 제공된 데이터 셋은 json이였기에 txt로 변환하는 과정이 필요하였고, 이를 [[공부/캠프/해커톤/해커톤_자료/cityscapes_yolov8-seg/convert.py|convert]]
을 통하여 txt파일로 변환하였음, 그 후 [[공부/캠프/해커톤/해커톤_자료/cityscapes_yolov8-seg/A.py|A]]와 [[공부/캠프/해커톤/해커톤_자료/cityscapes_yolov8-seg/data.yaml|data]]을 통하여 학습을 진행하였음

전체적으로 사람에 대한 데이터 셋이 적기에 학습이 부진하였음

그리고 길거리 사진으로 학습하였기에 서 있는 사람에 대해서만 학습을 진행하여 다른 자세에 대해서 학습률이 낮았음

추후 사용시 사람에 대해서는 다른 모델을 사용하여 합치는 것을 추천함
# 데이터 셋 주소
[https://www.cityscapes-dataset.com/]

# Label

| id  | label_name           | id  | label_name    | id  | label_name    |
| --- | -------------------- | --- | ------------- | --- | ------------- |
| 0   | road                 | 12  | rail track    | 24  | terrain       |
| 1   | sidewalk             | 13  | building      | 25  | sky           |
| 2   | person               | 14  | wall          | 26  | rider         |
| 3   | car                  | 15  | fence         | 27  | truck         |
| 4   | unlabeled            | 16  | guard rail    | 28  | bus           |
| 5   | ego vehicle          | 17  | bridge        | 29  | caravan       |
| 6   | rectification border | 18  | tunnel        | 30  | trailer       |
| 7   | out of roi           | 19  | pole          | 31  | train         |
| 8   | static               | 20  | polegroup     | 32  | motorcycle    |
| 9   | dynamic              | 21  | traffic light | 33  | bicycle       |
| 10  | ground               | 22  | traffic sign  | 34  | license plate |
| 11  | parking              | 23  | vegetation    |     |               |
# 학습 성능
## BOX_curve
### BoxF1_curve
![[첨부파일/BoxF1_curve.png]]
### BoxP_curve
![[첨부파일/BoxP_curve.png]]
### BoxPR_curve
![[첨부파일/BoxPR_curve.png]]
### BOX_R_curve
![[첨부파일/BoxR_curve.png]]
## confusion_matrix

### confusion_matrix_nomalized
![[첨부파일/confusion_matrix_normalized.png]]
### confusion_matrix
![[첨부파일/confusion_matrix.png]]
## labels
![[첨부파일/labels.jpg]]
## Mask_curve

### MaskF1_curve
![[첨부파일/MaskF1_curve.png]]
### MaskP_curve
![[첨부파일/MaskP_curve.png]]
### MaskPR_curve
![[첨부파일/MaskPR_curve.png]]
### MaskR_curve
![[첨부파일/MaskR_curve.png]]
### Results
![[첨부파일/results.png]]
### train_batch
![[첨부파일/train_batch0.jpg]]
![[첨부파일/train_batch1.jpg]]

![[첨부파일/train_batch2 1.jpg]]
![[첨부파일/train_batch16740.jpg]]
![[첨부파일/train_batch16741.jpg]]
![[첨부파일/train_batch16742.jpg]]


### val_batch
![[첨부파일/val_batch0_labels.jpg]]![[첨부파일/val_batch0_pred.jpg]]![[첨부파일/val_batch1_labels.jpg]]![[첨부파일/val_batch1_pred.jpg]]![[첨부파일/val_batch2_labels.jpg]]![[첨부파일/val_batch2_pred.jpg]]