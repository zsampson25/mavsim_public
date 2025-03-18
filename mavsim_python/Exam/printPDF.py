import pdfkit
# path_to_wkhtmltopdf = "uav/lib/python3.12/site-packages/wkhtmltopdf"  # Adjust path if needed
# config = pdfkit.configuration(wkhtmltopdf=path_to_wkhtmltopdf)

pdfkit.from_file("mavsim_public/mavsim_python/Exam/midterm_1.py", "midterm_1.pdf")
pdfkit.from_file("mavsim_public/mavsim_python/Exam/midterm_2.py", "midterm_2.pdf")
pdfkit.from_file("mavsim_public/mavsim_python/Exam/midterm_3_gainCalc.py", "midterm_3_gainCalc.pdf")
pdfkit.from_file("mavsim_public/mavsim_python/Exam/midterm_4.py", "midterm_4.pdf")
pdfkit.from_file("mavsim_public/mavsim_python/Exam/ufo_observer.py", "ufo_observer.pdf")



