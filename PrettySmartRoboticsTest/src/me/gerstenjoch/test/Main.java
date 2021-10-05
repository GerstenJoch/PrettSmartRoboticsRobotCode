package me.gerstenjoch.test;

import java.util.Scanner;
import java.util.concurrent.TimeUnit;

public class Main {
	private static Scanner sc = new Scanner(System.in);
	public static void main(String[] args) throws InterruptedException {
		while (true) {
		totallyATest();
		TimeUnit.SECONDS.sleep(3);
		}
	}
	public static void totallyATest() throws InterruptedException {
		System.out.println("Hello, type some random letters lol");
		String str1 = sc.nextLine();
		if (str1.contains("AB")) {
			System.out.println("Wow ur so good!");
		} else {
			System.out.println("Bruf");
		}
	}
}
