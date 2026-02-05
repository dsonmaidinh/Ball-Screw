# Dự án BallScrew Control

Tài liệu trong thư mục này giới thiệu hệ thống điều khiển BallScrew gồm phần cứng, phần mềm và firmware.

## Nội dung chính

- `App.png`: giao diện điều khiển và đồ thị hiển thị dữ liệu thời gian thực.
- `BallScrew.gif`: mô phỏng hoặc demo hoạt động cơ khí/điều khiển trong hệ thống.

## Mục tiêu

Dự án hướng tới xây dựng một hệ thống điều khiển vị trí/velocity cho cơ cấu BallScrew:

- STM32 F4 làm bộ điều khiển chính.
- Điều khiển PID kết hợp với encoder để theo dõi vị trí.
- Giao tiếp UART với phần mềm PC để hiển thị đồ thị và gửi lệnh.

## Hình ảnh minh họa

### Giao diện ứng dụng

![Giao diện ứng dụng điều khiển BallScrew](App.png)

### Demo hoạt động

![Demo hoạt động BallScrew](BallScrew.gif)

## Hướng dẫn nhanh

1. Mở firmware trong thư mục `firmware`.
2. Build và nạp vào STM32 F4.
3. Chạy ứng dụng PC trong `software/BallScrew.py`.
4. Kết nối cổng COM và bắt đầu điều khiển.

## Ghi chú

- Ảnh và GIF trong thư mục `docs` dùng để mô tả giao diện và trạng thái hệ thống.
- Tài liệu chính vẫn nằm trong README của từng phần `firmware`, `hardware`, `software`.
