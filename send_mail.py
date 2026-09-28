#!/usr/bin/env python3
"""Send status email for CloudForge-Analyzer Windows installer build task."""
import json, smtplib, sys
from email.mime.text import MIMEText
from email.header import Header
from email.utils import formataddr

CFG = json.load(open('/home/jamesyasr/.config/dsh-email-push-master/config.json'))['email']

def send(subject, body):
    msg = MIMEText(body, 'plain', 'utf-8')
    msg['Subject'] = Header(subject, 'utf-8')
    msg['From'] = formataddr((str(Header('CloudForge Build Agent', 'utf-8')), CFG['from']))
    msg['To'] = CFG['to']
    if CFG['useSsl']:
        s = smtplib.SMTP_SSL(CFG['smtpHost'], CFG['smtpPort'], timeout=30)
    else:
        s = smtplib.SMTP(CFG['smtpHost'], CFG['smtpPort'], timeout=30)
    try:
        s.login(CFG['from'], CFG['authCode'])
        proble = s.sendmail(CFG['from'], [CFG['to']], msg.as_string())
        print("SENT OK; refused:", proble)
    finally:
        s.quit()

if __name__ == '__main__':
    subject = sys.argv[1]
    body = sys.argv[2] if len(sys.argv) > 2 else subject
    send(subject, body)
